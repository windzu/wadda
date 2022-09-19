import os
import shutil
import json
from argparse import ArgumentParser
from rich.progress import track
import xml.etree.ElementTree as ET


class VOC2COCO:
    def __init__(self, path="./"):
        self.root_path = path

        (
            self.annos_path,
            self.train_set_path,
            self.val_set_path,
            self.test_set_path,
            self.labels_path,
            self.output_path,
        ) = self.init_path()

        self.label2id = self.init_label2id()

    def init_path(self):
        def init_coco_folder(path):
            # if path exit , delete it
            if os.path.exists(path):
                print(f"Delete {path}")
                shutil.rmtree(path)
            os.makedirs(os.path.join(path, "annotations"), exist_ok=True)
            os.makedirs(os.path.join(path, "train"), exist_ok=True)
            os.makedirs(os.path.join(path, "val"), exist_ok=True)
            os.makedirs(os.path.join(path, "test"), exist_ok=True)
            print("COCO folders initialized")

        annos_path = os.path.join(self.root_path, "Annotations")
        if not os.path.exists(annos_path):
            assert False, "Annotations path not exists"
        train_set_path = os.path.join(self.root_path, "ImageSets/Main/train.txt")
        if not os.path.exists(train_set_path):
            assert False, "train set path not exists"
        val_set_path = os.path.join(self.root_path, "ImageSets/Main/val.txt")
        if not os.path.exists(val_set_path):
            assert False, "val set path not exists"
        test_set_path = os.path.join(self.root_path, "ImageSets/Main/test.txt")
        if not os.path.exists(test_set_path):
            assert False, "test set path not exists"
        labels_path = os.path.join("./labels.txt")
        if not os.path.exists(labels_path):
            assert False, "labels path not exists"

        # init output path
        output_path = os.path.join(self.root_path, "coco")
        init_coco_folder(output_path)

        return (
            annos_path,
            train_set_path,
            val_set_path,
            test_set_path,
            labels_path,
            output_path,
        )

    def init_label2id(self):
        """Get label2id dict from labels file
        NOTE : label id start from 1
        Args:
            labels_path (str): Path to labels file
        Returns:
            label2id (Dict[str, int]): label2id dict
        """

        with open(self.labels_path, "r") as f:
            labels_str = f.read().split()
        labels_ids = list(range(1, len(labels_str) + 1))
        return dict(zip(labels_str, labels_ids))

    def run(self):
        self.__convert("train", self.train_set_path)
        self.__convert("val", self.val_set_path)
        self.__convert("test", self.test_set_path)
        print("Convert done")

    def __convert(self, dataset_type, ann_ids_path):
        """_summary_: convert voc to coco

        Args:
            dataset_type (str): train or val or test
            ann_ids_path (str): path to annotation ids file
        """
        # get annotation file path list
        with open(ann_ids_path, "r") as f:
            ann_ids = f.read().split()
            ann_paths = [os.path.join(self.annos_path, aid + ".xml") for aid in ann_ids]

        instances_filename = None
        if dataset_type == "train":
            instances_filename = "instances_train.json"
        elif dataset_type == "val":
            instances_filename = "instances_val.json"
        elif dataset_type == "test":
            instances_filename = "instances_test.json"
        else:
            assert False, "dataset type error"

        # convert xml to cocojson
        cocojson_path = os.path.join(self.output_path, "annotations", instances_filename)
        self.convert_xmls_to_cocojson(
            annotation_paths=ann_paths,
            label2id=self.label2id,
            output_jsonpath=cocojson_path,
        )
        # copy imgs to coco folder
        aim_path = os.path.join(self.output_path, dataset_type)
        for a_path in track(ann_paths):
            image_path = a_path.replace("Annotations", "JPEGImages").replace(".xml", ".jpg")
            shutil.copy(image_path, aim_path)

    @staticmethod
    def convert_xmls_to_cocojson(
        annotation_paths,
        label2id,
        output_jsonpath,
    ):
        """convert voc format xmls to coco format json

        Args:
            annotation_paths (List[str]): voc format annotation files paths
            label2id (Dict[str, int]): label to id dict
            output_jsonpath (str): output json file path
        """

        def get_image_info(annotation_root):
            """Get image info from annotation root
            NOTE : filename default end with .jpg
            """

            size = annotation_root.find("size")
            width = int(size.findtext("width"))
            height = int(size.findtext("height"))

            filename = annotation_root.findtext("filename")
            path = annotation_root.findtext("path")
            if filename:
                filename = filename.replace(".png", ".jpg")
            elif path:
                filename = os.path.basename(path).replace(".png", ".jpg")
            else:
                filename = None

            if filename:
                img_id = filename.split(".")[0]
            else:
                img_id = None

            image_info = {"file_name": filename, "height": height, "width": width, "id": img_id}
            return image_info

        def get_coco_annotation_from_obj(obj, label2id):
            label = obj.findtext("name")
            if label not in label2id:
                return None
            category_id = label2id[label]
            bndbox = obj.find("bndbox")
            xmin = int(float(bndbox.findtext("xmin"))) - 1
            ymin = int(float(bndbox.findtext("ymin"))) - 1
            xmax = int(float(bndbox.findtext("xmax")))
            ymax = int(float(bndbox.findtext("ymax")))
            if not (xmax > xmin and ymax > ymin):
                return None
            o_width = xmax - xmin
            o_height = ymax - ymin
            ann = {
                "area": o_width * o_height,
                "iscrowd": 0,
                "bbox": [xmin, ymin, o_width, o_height],
                "category_id": category_id,
                "ignore": 0,
                "segmentation": [],  # This script is not for segmentation
            }
            return ann

        output_json_dict = {"images": [], "type": "instances", "annotations": [], "categories": []}
        bnd_id = 1  # START_BOUNDING_BOX_ID, TODO input as args ?
        print("Start converting !")
        for a_path in track(annotation_paths):
            # Read annotation xml
            ann_tree = ET.parse(a_path)
            ann_root = ann_tree.getroot()

            img_info = get_image_info(annotation_root=ann_root)

            #
            if img_info["file_name"] is None:
                img_info["file_name"] = a_path.split("/")[-1].replace(".xml", ".jpg")
                img_info["id"] = img_info["file_name"].split(".")[0]
            img_id = img_info["id"]
            output_json_dict["images"].append(img_info)

            for obj in ann_root.findall("object"):
                ann = get_coco_annotation_from_obj(obj=obj, label2id=label2id)
                if ann is None:
                    continue
                ann.update({"image_id": img_id, "id": bnd_id})
                output_json_dict["annotations"].append(ann)
                bnd_id = bnd_id + 1

        for label, label_id in label2id.items():
            category_info = {"supercategory": "none", "id": label_id, "name": label}
            output_json_dict["categories"].append(category_info)

        with open(output_jsonpath, "w") as f:
            output_json = json.dumps(output_json_dict)
            f.write(output_json)


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, default="./", help="data root path")
    parser.add_argument("--pro", action="store_true", help="whether to enable professional mode")
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()
    voc2coc = VOC2COCO(path=args.path)
    voc2coc.run()


if __name__ == "__main__":
    main()
