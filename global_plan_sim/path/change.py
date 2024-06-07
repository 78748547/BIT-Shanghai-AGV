import re

# 定义读取文件内容的函数
def read_file(file_path):
    with open(file_path, 'r') as file:
        return file.read()

# 文件列表
file_list = [
    # 'global_plan_sim/path/path1.yaml',
    # 'global_plan_sim/path/path2.yaml',
    # 'global_plan_sim/path/path3.yaml'

    'global_plan_sim/path/path4.yaml',
    'global_plan_sim/path/path5.yaml',
    'global_plan_sim/path/path6.yaml',
    'global_plan_sim/path/path7.yaml'
]

# 读取第一个文件的内容
data1 = read_file(file_list[0])

# 提取第一个文件中的pose内容
pose1 = re.findall(r'(pose\d+:[\s\S]*?)(?=\n\S|$)', data1)

# 获取第一个文件中pose的最后一个序号
if pose1:
    last_index = max(int(re.search(r'pose(\d+)', p).group(1)) for p in pose1)
else:
    last_index = -1

# 初始化合并后的pose内容
all_poses = pose1

# 处理后续文件
for file_path in file_list[1:]:
    data = read_file(file_path)
    pose_data = re.findall(r'(pose\d+:[\s\S]*?)(?=\n\S|$)', data)
    
    # 更新pose中的序号
    new_pose_data = []
    for p in pose_data:
        last_index += 1
        new_pose_data.append(re.sub(r'pose\d+', f'pose{last_index}', p))
    
    all_poses.extend(new_pose_data)

# 将合并后的pose内容拼接到data1的pose部分
updated_data1 = re.sub(r'(pose\d+:[\s\S]*?)(?=\n\S|$)', '', data1).strip()
updated_data1 += '\n' + '\n'.join(all_poses)

# 保存新的path1.yaml文件
output_path = 'global_plan_sim/path/new_path2.yaml'
with open(output_path, 'w') as file:
    file.write(updated_data1)

print(f"拼接完成，新的文件已保存为 {output_path}")
