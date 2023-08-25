import os
import imageio.v2 as imageio
import cv2
from argparse import ArgumentParser
from rich.progress import track


class GIFGenerator:
    """将制定路径下的符合条件的文件夹中的文件转换为gif
    文件夹的不限定层级，其结构如下：
        data_root
        │   ├── 000
        │   └── 000
        │       └── 000
        │           ├── 0.jpg
        │           ├── 1.jpg
        │           └── 2.jpg
        └── 001
            └── 000
                ├── 0.jpg
                ├── 1.jpg
                └── 2.jpg
    将会在每个包含具体文件的文件夹同级目录生成一个名称为该文件夹的gif

    Args:
        path (str): data root path
    """

    def __init__(self, path):
        self.path = path

        self.resize_size = (640, 480)
        self.supported_file_type = ["jpg", "png", "bin", "pcd"]

    def run(self):
        folder_list = self.get_all_folders()
        folder_list.sort()
        for folder in track(folder_list):
            self.generate_gif(folder)

    def get_all_folders(self):
        """从根路径开始便利，收集所有满足条件的文件夹
        需要对包含文件的文件夹内所有文件进行检查，如果有不符合条件的文件，则不进行处理

        Returns:
            List: 所有满足条件的文件夹
        """
        folder_list = []
        for root, dirs, files in os.walk(self.path):
            if files:
                for file in files:
                    if file.endswith(tuple(self.supported_file_type)):
                        folder_list.append(root)
                        break
        return folder_list

    def generate_gif(self, folder):
        """generate gif
        Args:
            folder (str): folder name
        """
        # get current folder file type
        file_type = None
        for root, dirs, files in os.walk(os.path.join(self.path, folder)):
            if files:
                for file in files:
                    file_type = file.split(".")[-1]
                    if file_type in self.supported_file_type:
                        break
                    else:
                        raise Exception("Not supported file type: {}".format(file_type))
                break

        if file_type in ["jpg", "png"]:
            self.generate_gif_from_img(folder)
        elif file_type in ["bin", "pcd"]:
            self.generate_gif_from_pcd(folder)
        else:
            raise Exception("Not supported file type: {}".format(file_type))

    def generate_gif_from_img(self, folder):
        """将当前文件夹内的所有图片转换为gif,并保存到当前文件夹同级目录下

        Args:
            folder (str): image folder
        """
        save_path = os.path.join(self.path, folder + ".gif")
        img_path_list = [
            os.path.join(self.path, folder, f)
            for f in os.listdir(os.path.join(self.path, folder))
            if os.path.isfile(os.path.join(self.path, folder, f))
        ]
        img_path_list.sort()
        img_list = []
        for img_path in img_path_list:
            img = cv2.imread(img_path)
            img=img[...,::-1] # BGR2RGB
            img = cv2.resize(img, self.resize_size)
            img_list.append(img)
        imageio.mimsave(save_path, img_list, fps=10)

    def generate_gif_from_pcd(self, folder):
        # TODO : convert points to image
        pass


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, help="data root path")
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()

    # gif generator
    gif_generator = GIFGenerator(path=args.path)
    gif_generator.run()


if __name__ == "__main__":
    main()
