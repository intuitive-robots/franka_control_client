#!/bin/bash
#SBATCH --job-name=lerobot_convert
#SBATCH --time=01:00:00          
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=16       
#SBATCH --mem=64G                
#SBATCH --partition=dev_cpuonly      
#SBATCH --output=convert_%j.log

# ==============================================================================
# CONFIGURATION ZONE
# ==============================================================================
REPO_ID="plate_sponge_Sep_9"
TASK_INSTR="Pick up the sponge and place it on the plate. Go up, move to pick up the sponge on the plate, return it to its original position."
FPS=15
RESIZE_W=224
RESIZE_H=224
PUSH_TO_HUB=false

# Construct paths using the REPO_ID variable
RAW_DATA_DIR="/home/irl-admin/new_data_collection/plate"
OUTPUT_DATA_DIR="/home/irl-admin/new_data_collection/lerobot_meta/plate_sponge_Sep_9"
#!/bin/bash

source /home/irl-admin/miniconda3/etc/profile.d/conda.sh

conda activate mem_vla_eval
# 2. Run script with arguments
python /home/irl-admin/data_collect_scripts/franka_control_client/dataset_utils/convert_data_to_lerobot.py \
    --raw_dir "${RAW_DATA_DIR}" \
    --local_dir "${OUTPUT_DATA_DIR}" \
    --task_instr "${TASK_INSTR}" \
    --fps "${FPS}" \
    --resize_w "${RESIZE_W}" \
    --resize_h "${RESIZE_H}" \
    --push_to_hub "${PUSH_FLAG}"
# 3.upload to hugging face
# hf upload SSM-Memory-ICLR/pot_wait_30s_pepper_15hz \
#   /home/irl-admin/new_data_collection/lerobot_meta/pot \
#   . \
#   --repo-type dataset
