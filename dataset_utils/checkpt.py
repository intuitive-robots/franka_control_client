import torch
torch.set_printoptions(threshold=torch.inf,sci_mode=False)
pt = torch.load("/home/irl-admin/new_data_collection/human_demo_test/2026_04_25-12_12_31/FrankaPanda/gripper_current.pt")
# print(pt)
with open("current.txt","w") as f:
    f.write(str(pt))