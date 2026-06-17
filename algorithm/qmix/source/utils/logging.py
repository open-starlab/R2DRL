from collections import defaultdict
import logging
import numpy as np

try:
    import torch
except Exception:
    torch = None


def _to_float(v, *, key_for_warn=None, console_logger=None):
    if torch is not None and isinstance(v, torch.Tensor):
        if v.numel() == 1:
            out = float(v.detach().cpu().item())
        else:
            if console_logger is not None and key_for_warn is not None:
                console_logger.warning(
                    f"[Logger] '{key_for_warn}' 是多元素 Tensor {tuple(v.shape)}，将记录其 mean。"
                )
            out = float(v.detach().cpu().float().mean().item())
        if not np.isfinite(out) and console_logger is not None and key_for_warn is not None:
            console_logger.warning(f"[Logger] '{key_for_warn}' 值为 {out} (非有限数)!")
        return out

    if isinstance(v, np.ndarray):
        if v.size == 1:
            out = float(v.reshape(-1)[0])
        else:
            if console_logger is not None and key_for_warn is not None:
                console_logger.warning(
                    f"[Logger] '{key_for_warn}' 是多元素 numpy 数组 {v.shape}，将记录其 mean。"
                )
            out = float(v.astype(np.float32).mean())
        if not np.isfinite(out) and console_logger is not None and key_for_warn is not None:
            console_logger.warning(f"[Logger] '{key_for_warn}' 值为 {out} (非有限数)!")
        return out

    if isinstance(v, (list, tuple)):
        if len(v) == 0:
            return float("nan")
        return _to_float(np.asarray(v), key_for_warn=key_for_warn, console_logger=console_logger)

    try:
        out = float(v)
        if not np.isfinite(out) and console_logger is not None and key_for_warn is not None:
            console_logger.warning(f"[Logger] '{key_for_warn}' 值为 {out} (非有限数)!")
        return out
    except Exception:
        if console_logger is not None and key_for_warn is not None:
            console_logger.warning(f"[Logger] '{key_for_warn}' 无法转为 float，记为 NaN。类型：{type(v)}")
        return float("nan")


class _NullTBWriter:
    def add_scalar(self, *args, **kwargs):
        pass

    def add_histogram(self, *args, **kwargs):
        pass

    def add_text(self, *args, **kwargs):
        pass

    def flush(self):
        pass

    def close(self):
        pass


class Logger:
    def __init__(self, console_logger):
        self.console_logger = console_logger

        self.use_tb = False
        self.use_sacred = False
        self.use_hdf = False

        self.stats = defaultdict(lambda: [])
        self.tb_writer = _NullTBWriter()

    def setup_tb(self, directory_name):
        try:
            from torch.utils.tensorboard import SummaryWriter
            self.tb_writer = SummaryWriter(directory_name)
            self.use_tb = True
            self.console_logger.info(f"[Logger] TensorBoard log dir: {directory_name}")
        except Exception as e:
            self.console_logger.warning(f"[Logger] 无法创建 SummaryWriter：{e}")
            self.tb_writer = _NullTBWriter()
            self.use_tb = False

    def setup_sacred(self, sacred_run_dict):
        self.sacred_info = sacred_run_dict.info
        self.use_sacred = True

    def log_stat(self, key, value, t, to_sacred=True):
        value_f = _to_float(value, key_for_warn=key, console_logger=self.console_logger)
        self.stats[key].append((t, value_f))

        self.tb_writer.add_scalar(key, value_f, t)

        if self.use_sacred and to_sacred:
            if key in self.sacred_info:
                self.sacred_info[f"{key}_T"].append(t)
                self.sacred_info[key].append(value_f)
            else:
                self.sacred_info[f"{key}_T"] = [t]
                self.sacred_info[key] = [value_f]

    def print_recent_stats(self):
        if "episode" in self.stats and len(self.stats["episode"]) > 0:
            t_env, ep = self.stats["episode"][-1]
            log_str = "Recent Stats | t_env: {:>10} | Episode: {:>8}\n".format(t_env, ep)
        else:
            log_str = "Recent Stats\n"

        i = 0
        for (k, v) in sorted(self.stats.items()):
            if k == "episode":
                continue
            i += 1
            window = 5 if k != "epsilon" else 1
            vals = [_to_float(x[1]) for x in self.stats[k][-window:]]
            vals = [vv for vv in vals if vv == vv]
            item = "nan" if len(vals) == 0 else "{:.4f}".format(float(np.mean(vals)))
            log_str += "{:<25}{:>8}".format(k + ":", item)
            log_str += "\n" if i % 4 == 0 else "\t"

        self.console_logger.info(log_str)

    def close(self):
        try:
            self.tb_writer.flush()
            self.tb_writer.close()
        except Exception:
            pass


def get_logger():
    logger = logging.getLogger()
    logger.handlers = []
    ch = logging.StreamHandler()
    formatter = logging.Formatter('[%(levelname)s %(asctime)s] %(name)s %(message)s', '%H:%M:%S')
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    logger.setLevel('DEBUG')
    return logger