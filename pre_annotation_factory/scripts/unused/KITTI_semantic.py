import os
import random
import shutil
import argparse

def main():
    # 1. 设置命令行参数解析
    parser = argparse.ArgumentParser(description="同步 KITTI 格式的图像与语义标注，并生成 train/val 划分文件。")
    parser.add_argument("-d", "--dataset_dir", type=str, required=True,
                        help="数据集根目录的路径 (例如: ~/Data/AutoAnnotation/.../Semantics_Annotation_...)")
    parser.add_argument("-r", "--ratio", type=float, default=0.8,
                        help="训练集所占比例，默认 0.8 (即 80%)")
    parser.add_argument("-s", "--seed", type=int, default=42,
                        help="随机种子，用于打乱数据集，默认 42")
    
    args = parser.parse_args()

    # 处理路径中的 "~" 并获取绝对路径
    base_dir = os.path.abspath(os.path.expanduser(args.dataset_dir))

    if not os.path.exists(base_dir):
        print(f"-> 错误: 找不到指定的数据集路径 '{base_dir}'，请检查输入。")
        return

    print(f"-> 正在处理数据集: {base_dir}")

    # 2. 定义具体的子文件夹路径
    semantic_dir = os.path.join(base_dir, "training", "semantic")
    image_dir = os.path.join(base_dir, "training", "image_2")
    imagesets_dir = os.path.join(base_dir, "ImageSets")
    unused_dir = os.path.join(base_dir, "training", "image_2_unused")

    # 检查核心文件夹是否存在
    if not os.path.exists(semantic_dir) or not os.path.exists(image_dir):
        print("-> 错误: 目标路径下缺失 'training/semantic' 或 'training/image_2' 文件夹。")
        return

    # 3. 获取所有有语义标注的基准文件名（不带后缀）
    semantic_files = [f for f in os.listdir(semantic_dir) if os.path.isfile(os.path.join(semantic_dir, f))]
    valid_basenames = set(os.path.splitext(f)[0] for f in semantic_files)
    print(f"-> 在 semantic 文件夹中找到 {len(valid_basenames)} 个标注文件。")

    # 4. 同步 image_2 文件夹（移除多余图片）
    os.makedirs(unused_dir, exist_ok=True)
    image_files = [f for f in os.listdir(image_dir) if os.path.isfile(os.path.join(image_dir, f))]
    removed_count = 0

    for img in image_files:
        basename = os.path.splitext(img)[0]
        if basename not in valid_basenames:
            shutil.move(os.path.join(image_dir, img), os.path.join(unused_dir, img))
            removed_count += 1

    if removed_count > 0:
        print(f"-> 已将 {removed_count} 个没有对应标注的图像移动到了 {unused_dir}。")
    else:
        print("-> image_2 文件夹已是同步状态。")

    # 5. 二次校验：确保 semantic 里的文件在 image_2 里都有原图
    current_image_basenames = set(os.path.splitext(f)[0] for f in os.listdir(image_dir))
    missing_images = valid_basenames - current_image_basenames
    if missing_images:
        print(f"-> 警告: 有 {len(missing_images)} 个标注文件缺失对应的原图！将从列表中剔除它们。")
        valid_basenames = valid_basenames & current_image_basenames

    # 6. 划分数据集
    valid_basenames_list = sorted(list(valid_basenames)) 
    random.seed(args.seed)
    random.shuffle(valid_basenames_list)

    split_idx = int(len(valid_basenames_list) * args.ratio)
    train_names = valid_basenames_list[:split_idx]
    val_names = valid_basenames_list[split_idx:]

    # 7. 生成 train.txt 和 val.txt
    os.makedirs(imagesets_dir, exist_ok=True)

    with open(os.path.join(imagesets_dir, "train.txt"), "w") as f:
        f.write("\n".join(train_names) + "\n")

    with open(os.path.join(imagesets_dir, "val.txt"), "w") as f:
        f.write("\n".join(val_names) + "\n")

    print(f"-> 数据集划分完成！\n   训练集 (train): {len(train_names)} 张\n   验证集 (val): {len(val_names)} 张")
    print(f"-> txt 文件已成功写入至 {imagesets_dir} 目录下。")

if __name__ == "__main__":
    main()