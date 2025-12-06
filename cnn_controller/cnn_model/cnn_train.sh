#!/bin/sh

# define a base directory for the experiment
DL_EXP=`pwd`;

# scripts files
DL_SCRIPTS=$DL_EXP;

# output directory
DL_OUT="$DL_EXP/output";
if [ ! -d $DL_OUT ]; then
      mkdir $DL_OUT
fi

# define parameter file
DL_PARAM_DIR="$DL_EXP/../params";
DL_PARAM_FILE="$DL_PARAM_DIR/hyperparameters.yaml";

# define the output directories for training/decoding/scoring
DL_TRAIN_ODIR="$DL_EXP/model";
if [ ! -d $DL_TRAIN_ODIR ]; then
      mkdir $DL_TRAIN_ODIR
fi

# define dataset directories
DL_DATA="$DL_EXP/turtlebot3_dataset";
DL_DATA_TRAIN="$DL_DATA/train";
DL_DATA_EVAL="$DL_DATA/dev";

# execute training: training must always be run
echo "======= start of training ======="
echo "Loading data from $DL_DATA_TRAIN"
python3 $DL_SCRIPTS/cnn_train.py $DL_PARAM_FILE $DL_TRAIN_ODIR $DL_DATA_TRAIN $DL_DATA_EVAL | tee $DL_OUT/00_train.log | \
      grep "reading\|Step\|Average\|Warning\|Error" 
echo "Finished training on $DL_DATA_TRAIN"

echo "Saving model weights to parameter directory"
cp $DL_TRAIN_ODIR/model_final.pth $DL_PARAM_DIR/cnn_model_weights.pth
echo "======= end of training ======="
