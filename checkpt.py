import torch
torch.set_printoptions(threshold=torch.inf,sci_mode=False)
pt = torch.load("/home/irl-admin/new_data_collection/pepper_40HZ/2026_02_13-09_15_30/FrankaPanda/joint_pos.pt")
# print(pt)
with open("qs.txt","w") as f:
    f.write(str(pt))