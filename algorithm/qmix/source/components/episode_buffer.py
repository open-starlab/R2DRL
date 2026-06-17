import torch as th
import numpy as np
from types import SimpleNamespace as SN


class EpisodeBatch:
    def __init__(self,
                 scheme,
                 groups,
                 batch_size,
                 max_seq_length,
                 data=None,
                 preprocess=None,
                 device="cpu"):
        self.scheme = scheme.copy() #字段规格表（每个键的形状/类型/是否有group/是否是episode常量）
        self.groups = groups #各 group 的规模（如 {"agents": n_agents}）
        self.batch_size = batch_size #一次能容纳多少条 episode。
        self.max_seq_length = max_seq_length #每条 episode 的最长时间步 T。
        self.preprocess = {} if preprocess is None else preprocess #预处理规则（如把 actions 转成 actions_onehot）。
        self.device = device

        if data is not None:
            self.data = data #若外部已经准备好了 data（含 transition_data/episode_data），直接接管。
        else:
            self.data = SN()
            self.data.transition_data = {} #时序序列
            self.data.episode_data = {} #静态设置
            self._setup_data(self.scheme, self.groups, batch_size, max_seq_length, self.preprocess)

    def _setup_data(self, scheme, groups, batch_size, max_seq_length, preprocess):
        # Generate a one-hot representation of action based on the scheme
        if preprocess is not None:
            for k in preprocess:
                assert k in scheme #源字段名 k必须已经在 scheme 中存在
                new_k = preprocess[k][0]#派生字段名，例如 "actions_onehot"
                transforms = preprocess[k][1]#变换器列表

                vshape = self.scheme[k]["vshape"]
                dtype = self.scheme[k]["dtype"]
                for transform in transforms:
                    vshape, dtype = transform.infer_output_info(vshape, dtype)

                self.scheme[new_k] = {
                    "vshape": vshape,
                    "dtype": dtype
                }
                if "group" in self.scheme[k]:
                    self.scheme[new_k]["group"] = self.scheme[k]["group"]
                if "episode_const" in self.scheme[k]:
                    self.scheme[new_k]["episode_const"] = self.scheme[k]["episode_const"]

        assert "filled" not in scheme, '"filled" is a reserved key for masking.'
        scheme.update({
            "filled": {"vshape": (1,), "dtype": th.long},
        })

        for field_key, field_info in scheme.items():
            assert "vshape" in field_info, "Scheme must define vshape for {}".format(field_key)
            vshape = field_info["vshape"]
            episode_const = field_info.get("episode_const", False)
            group = field_info.get("group", None)
            dtype = field_info.get("dtype", th.float32)

            if isinstance(vshape, int):
                vshape = (vshape,)

            if group:
                assert group in groups, "Group {} must have its number of members defined in _groups_".format(group)
                shape = (groups[group], *vshape)
            else:
                shape = vshape

            if episode_const:
                self.data.episode_data[field_key] = th.zeros((batch_size, *shape), dtype=dtype, device=self.device)
            else:
                self.data.transition_data[field_key] = th.zeros((batch_size, max_seq_length, *shape), dtype=dtype, device=self.device)

    def extend(self, scheme, groups=None):
        self._setup_data(scheme, self.groups if groups is None else groups, self.batch_size, self.max_seq_length)

    def to(self, device):
        for k, v in self.data.transition_data.items():
            self.data.transition_data[k] = v.to(device)
        for k, v in self.data.episode_data.items():
            self.data.episode_data[k] = v.to(device)
        self.device = device

    def update(self, data, bs=slice(None), ts=slice(None), mark_filled=True):
        """
        Update values in the EpisodeBatch with new data.

        Args:
            data (dict): A dictionary where keys are feature names (e.g., "state", "actions")
                         and values are numpy arrays or tensors of new data.
            bs (slice or list): Batch indices to update. Default is all batches.
            ts (slice or list): Time indices to update. Default is all timesteps.
            mark_filled (bool): If True, marks the specified timesteps as filled
                                (used for masking in training).

        Notes:
            - This function updates either `transition_data` (time-dependent features)
              or `episode_data` (episode-level features) depending on the key.
            - Supports preprocessing transformations (e.g., converting actions to one-hot).
        """

        # Parse and normalize the batch/time slices into a tuple of slices
        slices = self._parse_slices((bs, ts))

        # Loop through each field to update
        for k, v in data.items():
            # If key belongs to transition-level data (time-series data)
            if k in self.data.transition_data:
                target = self.data.transition_data
                if mark_filled:
                    # Mark these timesteps as filled for masking
                    target["filled"][slices] = 1
                    mark_filled = False  # Only mark once per update call
                _slices = slices

            # If key belongs to episode-level data (constant for the whole episode)
            elif k in self.data.episode_data:
                target = self.data.episode_data
                _slices = slices[0]  # Episode data doesn't need time index

            # If the key isn't found, throw an error
            else:
                raise KeyError(f"{k} not found in transition or episode data")

            # Convert incoming data to a torch tensor with the correct dtype and device
            dtype = self.scheme[k].get("dtype", th.float32)
            v = self._to_tensor_fast(v, dtype=dtype, device=self.device)
            
            # dest = target[k][_slices]

            # # 🔍 如果形状不一样，先打个 log 再走 _check_safe_view
            # if tuple(v.shape) != tuple(dest.shape):
            #     print(
            #         f"[EpisodeBatch.update][DEBUG] key={k}, "
            #         f"bs={bs}, ts={ts}, "
            #         f"src_shape={tuple(v.shape)}, dest_shape={tuple(dest.shape)}"
            #     )

            # Ensure the tensor can safely be reshaped into the target shape
            self._check_safe_view(v, target[k][_slices])

            # Assign the reshaped data to the target tensor
            target[k][_slices] = v.view_as(target[k][_slices])

            # Apply preprocessing transforms if this key has them defined
            if k in self.preprocess:
                new_k = self.preprocess[k][0]  # Name of the transformed feature
                v = target[k][_slices]
                for transform in self.preprocess[k][1]:
                    v = transform.transform(v)  # Apply transformation(s)
                target[new_k][_slices] = v.view_as(target[new_k][_slices])

    def _to_tensor_fast(self, v, dtype, device):
        # 1) 如果是 [np.ndarray, np.ndarray, ...] ，先堆叠为一个 np.ndarray
        if isinstance(v, (list, tuple)) and len(v) > 0 and isinstance(v[0], np.ndarray):
            v = np.asarray(v)

        # 2) np.ndarray -> torch（零拷贝视图；再 to 到目标 device/dtype）
        if isinstance(v, np.ndarray):
            # 确保连续，避免 from_numpy 视图不连续导致后续异常
            v = np.ascontiguousarray(v)
            return th.from_numpy(v).to(device=device, dtype=dtype)

        # 3) 已经是 tensor -> 直接 to（避免二次复制 & 警告）
        if th.is_tensor(v):
            return v.to(device=device, dtype=dtype)

        # 4) 其它标量/小容器 -> as_tensor（尽量复用内存）
        return th.as_tensor(v, dtype=dtype, device=device)


    def _check_safe_view(self, v, dest):
        idx = len(v.shape) - 1
        for s in dest.shape[::-1]:
            if v.shape[idx] != s:
                if s != 1:
                    print(
                        f"[EpisodeBatch._check_safe_view][DEBUG] "
                        f"v.shape={tuple(v.shape)}, dest.shape={tuple(dest.shape)}")
                    raise ValueError("Unsafe reshape of {} to {}".format(v.shape, dest.shape))
            else:
                idx -= 1

    def __getitem__(self, item):
        if isinstance(item, str):
            if item in self.data.episode_data:
                return self.data.episode_data[item]
            elif item in self.data.transition_data:
                return self.data.transition_data[item]
            else:
                raise ValueError
        elif isinstance(item, tuple) and all([isinstance(it, str) for it in item]):
            new_data = self._new_data_sn()
            for key in item:
                if key in self.data.transition_data:
                    new_data.transition_data[key] = self.data.transition_data[key]
                elif key in self.data.episode_data:
                    new_data.episode_data[key] = self.data.episode_data[key]
                else:
                    raise KeyError("Unrecognised key {}".format(key))

            # Update the scheme to only have the requested keys
            new_scheme = {key: self.scheme[key] for key in item}
            new_groups = {self.scheme[key]["group"]: self.groups[self.scheme[key]["group"]]
                          for key in item if "group" in self.scheme[key]}
            ret = EpisodeBatch(new_scheme, new_groups, self.batch_size, self.max_seq_length, data=new_data, device=self.device)
            return ret
        else:
            item = self._parse_slices(item)
            new_data = self._new_data_sn()
            for k, v in self.data.transition_data.items():
                new_data.transition_data[k] = v[item]
            for k, v in self.data.episode_data.items():
                new_data.episode_data[k] = v[item[0]]

            ret_bs = self._get_num_items(item[0], self.batch_size)
            ret_max_t = self._get_num_items(item[1], self.max_seq_length)

            ret = EpisodeBatch(self.scheme, self.groups, ret_bs, ret_max_t, data=new_data, device=self.device)
            return ret

    def _get_num_items(self, indexing_item, max_size):
        if isinstance(indexing_item, list) or isinstance(indexing_item, np.ndarray):
            return len(indexing_item)
        elif isinstance(indexing_item, slice):
            _range = indexing_item.indices(max_size)
            return 1 + (_range[1] - _range[0] - 1)//_range[2]

    def _new_data_sn(self):
        new_data = SN()
        new_data.transition_data = {}
        new_data.episode_data = {}
        return new_data

    def _parse_slices(self, items):
        parsed = []
        # Only batch slice given, add full time slice
        if (isinstance(items, slice)  # slice a:b
            or isinstance(items, int)  # int i
            or (isinstance(items, (list, np.ndarray, th.LongTensor, th.cuda.LongTensor)))  # [a,b,c]
            ):
            items = (items, slice(None))

        # Need the time indexing to be contiguous
        if isinstance(items[1], list):
            raise IndexError("Indexing across Time must be contiguous")

        for item in items:
            #TODO: stronger checks to ensure only supported options get through
            if isinstance(item, int):
                # Convert single indices to slices
                parsed.append(slice(item, item+1))
            else:
                # Leave slices and lists as is
                parsed.append(item)
        return parsed

    def max_t_filled(self):
        return th.sum(self.data.transition_data["filled"], 1).max(0)[0]

    def __repr__(self):
        return "EpisodeBatch. Batch Size:{} Max_seq_len:{} Keys:{} Groups:{}".format(self.batch_size,
                                                                                     self.max_seq_length,
                                                                                     self.scheme.keys(),
                                                                                     self.groups.keys())


class ReplayBuffer(EpisodeBatch):
    def __init__(self, scheme, groups, buffer_size, max_seq_length, preprocess=None, device="cpu"):
        super(ReplayBuffer, self).__init__(scheme, groups, buffer_size, max_seq_length, preprocess=preprocess, device=device)
        self.buffer_size = buffer_size  # same as self.batch_size but more explicit
        self.buffer_index = 0
        self.episodes_in_buffer = 0

    def insert_episode_batch(self, ep_batch):
        if self.buffer_index + ep_batch.batch_size <= self.buffer_size:
            self.update(ep_batch.data.transition_data,
                        slice(self.buffer_index, self.buffer_index + ep_batch.batch_size),
                        slice(0, ep_batch.max_seq_length),
                        mark_filled=False)
            self.update(ep_batch.data.episode_data,
                        slice(self.buffer_index, self.buffer_index + ep_batch.batch_size))
            self.buffer_index = (self.buffer_index + ep_batch.batch_size)
            self.episodes_in_buffer = max(self.episodes_in_buffer, self.buffer_index)
            self.buffer_index = self.buffer_index % self.buffer_size
            assert self.buffer_index < self.buffer_size
        else:
            buffer_left = self.buffer_size - self.buffer_index
            self.insert_episode_batch(ep_batch[0:buffer_left, :])
            self.insert_episode_batch(ep_batch[buffer_left:, :])

    def can_sample(self, batch_size):
        return self.episodes_in_buffer >= batch_size

    def sample(self, batch_size):
        assert self.can_sample(batch_size)
        if self.episodes_in_buffer == batch_size:
            return self[:batch_size]
        else:
            # Uniform sampling only atm
            ep_ids = np.random.choice(self.episodes_in_buffer, batch_size, replace=False)
            return self[ep_ids]

    def __repr__(self):
        return "ReplayBuffer. {}/{} episodes. Keys:{} Groups:{}".format(self.episodes_in_buffer,
                                                                        self.buffer_size,
                                                                        self.scheme.keys(),
                                                                        self.groups.keys())

