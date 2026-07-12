@echo off
cd /d "%~dp0"
python test_model.py --weights weights/bc_weights_v7.npz --barn-zip datasets/BARN_dataset.zip --world 0
pause