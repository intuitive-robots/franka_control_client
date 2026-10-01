import torch
torch.set_printoptions(threshold=torch.inf,sci_mode=False)
pt = torch.load("/home/irl-admin/new_data_collection/robot_state_record_static/2026_07_22-13_39_54/FrankaPanda/joint_pos.pt")
# print(pt)
with open("initial_gello_joint_pos.txt","w") as f:
    f.write(str(pt))