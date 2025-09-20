#\!/bin/bash
# 这是一个只上传deepseek_arm_control到GitHub的脚本
echo "开始上传deepseek_arm_control到GitHub..."
echo "1. 初始化git仓库..."
git init
git remote add origin https://github.com/SummerPer/LLM-Based-Control-of-Robotic-Arms
echo "2. 复制deepseek_arm_control项目文件..."
cp -r /home/ubuntu22/ws_moveit2/src/deepseek_arm_control .
echo "3. 创建.gitignore文件..."
echo -e "build/\ninstall/\nlog/\n*.pyc\n__pycache__/\n.DS_Store\n*.swp\n*.swo\n*~" > .gitignore
echo "4. 添加并提交文件..."
git add .
git commit -m "Add deepseek_arm_control package"
echo "5. 推送到GitHub..."
echo "提示：当系统提示输入密码时，请输入您的个人访问令牌"
git push -u origin master
