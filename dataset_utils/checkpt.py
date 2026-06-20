import torch
torch.set_printoptions(threshold=torch.inf,sci_mode=False)
pt = torch.load("/home/irl-admin/new_data_collection/pepper_100hz_cam_25hz/2026_06_18-16_18_27/FrankaPanda/ee_pos.pt")
# print(pt)
with open("ee_pos.txt","w") as f:
    f.write(str(pt))