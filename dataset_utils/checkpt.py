import torch
torch.set_printoptions(threshold=torch.inf,sci_mode=False)
pt = torch.load("/home/irl-admin/new_data_collection/new_scarf_100hz_cam_25hz/2026_03_27-19_24_42/FrankaPanda/joint_pos.pt")
# print(pt)
with open("qs.txt","w") as f:
    f.write(str(pt))