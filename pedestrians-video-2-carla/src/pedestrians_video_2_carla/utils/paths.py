import os
import re


def get_run_id_from_log_dir(log_dir):
    """
    Get the run id from a log directory.
    """
    run_re = re.compile(r"^.*?(([a-z]+-?)?[a-z0-9]+)(\:v[0-9]+)?$")
    run_id = run_re.match(log_dir.split(os.path.sep)[-1]).group(1)
    return run_id


def get_run_id_from_checkpoint_path(ckpt_path):
    """
    Get the run id from a checkpoint path.
    """
    path_parts = ckpt_path.split(os.path.sep)
    idx = -3 if 'checkpoints' in path_parts else -2
    return get_run_id_from_log_dir(path_parts[idx])


def resolve_ckpt_path(ckpt_path):
    if ckpt_path.startswith("wandb://"):
        import wandb
        artifact_path = ckpt_path[len('wandb://'):]
        artifact = wandb.run.use_artifact(artifact_path, type='model')
        artifact_dir = artifact.download()
        ckpt_path = os.path.join(artifact_dir, "model.ckpt")
    elif ckpt_path.startswith("file://"):
        ckpt_path = ckpt_path[len('file://'):]
    return ckpt_path
