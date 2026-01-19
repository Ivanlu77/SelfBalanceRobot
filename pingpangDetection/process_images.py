import os
import cv2
import numpy as np
from pathlib import Path

def process_images(input_dir, output_dir, target_size=(320, 240)):
    """
    处理图片：调整大小并重命名，保持宽高比
    :param input_dir: 输入图片目录
    :param output_dir: 输出图片目录
    :param target_size: 目标尺寸，默认 320x240
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取所有jpg文件
    image_files = sorted([f for f in os.listdir(input_dir) if f.lower().endswith(('.jpg', '.jpeg'))])
    
    print(f"找到 {len(image_files)} 个图片文件")
    
    # 处理每个图片
    for idx, img_file in enumerate(image_files, 1):
        # 构建输入输出路径
        input_path = os.path.join(input_dir, img_file)
        output_path = os.path.join(output_dir, f"{idx:03d}.jpg")
        
        try:
            # 读取图片
            img = cv2.imread(input_path)
            if img is None:
                print(f"无法读取图片: {img_file}")
                continue
            
            # 获取原始尺寸
            h, w = img.shape[:2]
            
            # 计算缩放比例
            scale = min(target_size[0]/w, target_size[1]/h)
            new_size = (int(w*scale), int(h*scale))
            
            # 调整大小，保持宽高比
            resized_img = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
            
            # 创建黑色背景
            final_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
            
            # 计算居中位置
            x_offset = (target_size[0] - new_size[0]) // 2
            y_offset = (target_size[1] - new_size[1]) // 2
            
            # 将调整后的图片放在黑色背景中央
            final_img[y_offset:y_offset+new_size[1], x_offset:x_offset+new_size[0]] = resized_img
            
            # 保存图片
            cv2.imwrite(output_path, final_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            print(f"处理完成: {img_file} -> {os.path.basename(output_path)}")
            print(f"原始尺寸: {w}x{h}, 新尺寸: {new_size[0]}x{new_size[1]}")
            
        except Exception as e:
            print(f"处理图片 {img_file} 时出错: {str(e)}")
    
    print("\n处理完成！")
    print(f"处理后的图片保存在: {output_dir}")

if __name__ == "__main__":
    # 设置输入输出目录
    input_dir = "raw_images"  # 原始图片目录
    train_dir = "original data/images/train"  # 训练集目录
    val_dir = "original data/images/val"  # 验证集目录
    
    # 确保输入目录存在
    if not os.path.exists(input_dir):
        os.makedirs(input_dir)
        print(f"请将原始图片放入 {input_dir} 目录")
        exit()
    
    # 处理训练集图片（80%）
    image_files = sorted([f for f in os.listdir(input_dir) if f.lower().endswith(('.jpg', '.jpeg'))])
    train_count = int(len(image_files) * 0.8)
    
    # 创建临时目录用于处理训练集图片
    temp_train_dir = "temp_train"
    os.makedirs(temp_train_dir, exist_ok=True)
    
    # 复制训练集图片到临时目录
    for i, img_file in enumerate(image_files[:train_count]):
        src = os.path.join(input_dir, img_file)
        dst = os.path.join(temp_train_dir, img_file)
        os.system(f'copy "{src}" "{dst}"')
    
    # 处理训练集图片
    process_images(temp_train_dir, train_dir)
    
    # 创建临时目录用于处理验证集图片
    temp_val_dir = "temp_val"
    os.makedirs(temp_val_dir, exist_ok=True)
    
    # 复制验证集图片到临时目录
    for i, img_file in enumerate(image_files[train_count:]):
        src = os.path.join(input_dir, img_file)
        dst = os.path.join(temp_val_dir, img_file)
        os.system(f'copy "{src}" "{dst}"')
    
    # 处理验证集图片
    process_images(temp_val_dir, val_dir)
    
    # 清理临时目录
    import shutil
    shutil.rmtree(temp_train_dir)
    shutil.rmtree(temp_val_dir)
    
    print("\n所有图片处理完成！")
    print(f"训练集图片保存在: {train_dir}")
    print(f"验证集图片保存在: {val_dir}") 