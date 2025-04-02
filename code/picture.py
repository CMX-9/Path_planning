import cv2
import numpy as np
import torch
from torchvision import transforms
from PIL import Image
from u2net import U2NET  # 导入 U2NET 类

# 加载预训练的U²-Net模型
def load_u2net_model():
    model = U2NET()  # 实例化 U2NET 模型
    model_path = r"D:\autonomous-cwk2\u2net.pth"  # 确保路径正确
    # 加载模型参数
    model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
    model.eval()  # 切换到评估模式
    return model

# 图像预处理
def preprocess_image(image):
    transform = transforms.Compose([
        transforms.Resize((320, 320)),  # U²-Net 输入尺寸
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    return transform(image).unsqueeze(0)

# 图像后处理
def postprocess_mask(mask):
    mask = mask.squeeze().cpu().numpy()
    mask = (mask * 255).astype(np.uint8)
    mask = cv2.resize(mask, (original_width, original_height))
    return mask

# 使用U²-Net进行人像分割
def segment_image(model, image_path):
    global original_height, original_width
    image = Image.open(image_path).convert("RGB")
    original_width, original_height = image.size
    
    # 预处理
    input_tensor = preprocess_image(image)
    
    # 推理
    with torch.no_grad():
        output = model(input_tensor)
    
    # 后处理
    mask = postprocess_mask(output[0])  # 使用主输出 (d0)
    return mask

# 替换背景颜色
def replace_background(image_path, mask, output_path):
    image = cv2.imread(image_path)
    if image is None:
        print("错误：图片读取失败")
        return
    
    # 采样相邻背景色
    border_mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=5) - mask
    sampled_color = cv2.mean(image, mask=border_mask)[:3]
    
    # 替换背景
    result = image.copy()
    result[mask == 0] = sampled_color  # 将背景区域替换为采样颜色
    
    # 保存结果
    cv2.imwrite(output_path, result)

# 主函数
def main():
    # 加载模型
    model = load_u2net_model()
    
    # 输入输出路径
    input_path = r"C:\Users\14288\Desktop\wb\wubin.png"
    output_path = r"C:\Users\14288\Desktop\wb\output_segmented.png"
    
    # 分割图像
    mask = segment_image(model, input_path)
    
    # 替换背景
    replace_background(input_path, mask, output_path)
    print(f"处理完成，结果已保存到：{output_path}")

if __name__ == "__main__":
    main()