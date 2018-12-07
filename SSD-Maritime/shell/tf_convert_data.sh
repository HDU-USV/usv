＃filename = tf_convert_data.sh
#!/bin/bash
#This is a shell script to convert Pascal VOC datasets(2007 and 2012) into TF-Records only.

#Directory where the original dataset is stored
DATASET_DIR=/home/luobicangqiong/data/VOC2007_train/VOCdevkit/VOC2007/

#Output directory where to store TFRecords files
OUTPUT_DIR=/home/luobicangqiong/data/VOC2007_train/VOCdevkit/VOC2007_tfrecord/

python ../tf_convert_data.py \
    --dataset_name=pascalvoc \
    --dataset_dir=${DATASET_DIR} \
    --output_name=voc_2007_train \
    --output_dir=${OUTPUT_DIR}
