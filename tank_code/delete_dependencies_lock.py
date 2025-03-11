import os
import fnmatch

def delete_matching_files(directory, patterns):
    for root, dirs, files in os.walk(directory):
        for file in files:
            for pattern in patterns:
                if fnmatch.fnmatch(file, pattern):
                    file_path = os.path.join(root, file)
                    try:
                        os.remove(file_path)
                        print(f"已删除文件: {file_path}")
                    except OSError as e:
                        print(f"删除文件 {file_path} 时出错: {e}")
                    
patterns = ['*.component_hash','dependencies.lock']
directory = '.'

delete_matching_files(directory,patterns)