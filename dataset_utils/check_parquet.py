from pathlib import Path
p = Path("/home/irl-admin/Downloads/file-000.parquet")
data = p.read_bytes()
print(data[:4], data[-4:])
