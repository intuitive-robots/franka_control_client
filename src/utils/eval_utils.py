import torch
from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.factory import IMAGENET_STATS


def load_dataset_meta(cfg: TrainPipelineConfig):
    """Load the dataset meta information including normalization stats exactly as during training."""
    ds_meta = LeRobotDatasetMetadata(
        cfg.dataset.repo_id, root=cfg.dataset.root, revision=cfg.dataset.revision
    )
    print('debug:',cfg.dataset.root)
    if cfg.dataset.use_imagenet_stats:
        for key in ds_meta.camera_keys:
            for stats_type, stats in IMAGENET_STATS.items():
                ds_meta.stats[key][stats_type] = torch.tensor(stats, dtype=torch.float32)
    return ds_meta


def test_flash():
    import flash_attn
    from flash_attn import flash_attn_func

    print("Flash Attention version:", flash_attn.__version__)

    q, k, v = [torch.randn(2, 16, 64, 64, dtype=torch.bfloat16).cuda() for _ in range(3)]

    try:
        out = flash_attn_func(q, k, v, causal=False)
        print("Flash Attention executed successfully. Output shape:", out.shape)
    except Exception as e:
        print("Error during Flash Attention execution:", str(e))
