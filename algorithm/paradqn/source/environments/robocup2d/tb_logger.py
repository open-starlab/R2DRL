from torch.utils.tensorboard import SummaryWriter
import os
import time

class TBLogger:
    def __init__(self, log_dir: str, enabled: bool):
        self.enabled = enabled
        self.writer = None

        if self.enabled:
            run_name = time.strftime("%Y%m%d-%H%M%S")
            run_dir = os.path.join(log_dir, run_name)

            os.makedirs(run_dir, exist_ok=True)

            self.writer = SummaryWriter(log_dir=run_dir)
            print(f"[TensorBoard] logdir = {run_dir}")

    def add_scalar(self, tag: str, value, step: int):
        if self.writer is not None:
            self.writer.add_scalar(tag, value, step)

    def add_scalars(self, main_tag: str, tag_scalar_dict: dict, step: int):
        if self.writer is not None:
            self.writer.add_scalars(main_tag, tag_scalar_dict, step)

    def flush(self):
        if self.writer is not None:
            self.writer.flush()

    def close(self):
        if self.writer is not None:
            self.writer.close()