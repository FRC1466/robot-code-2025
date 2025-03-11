@echo off
echo Setting up FRC ML environment using Mamba...

REM Check if Mamba is installed
call mamba --version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo Mamba not found. Please install Mambaforge or Miniforge first.
    echo Download from: https://github.com/conda-forge/miniforge#mambaforge
    exit /b 1
)

REM Create environment from YAML file
echo Creating Mamba environment from environment.yml...
call mamba env create -f environment.yml

if %ERRORLEVEL% NEQ 0 (
    echo Failed to create environment. Please check errors above.
    exit /b 1
)

echo.
echo Environment setup complete!
echo.
echo To activate the environment, run:
echo   mamba activate frc-ml
echo.
echo To run the autonomous system:
echo   python frc_autonomous_system.py
echo.
echo To test just the depth estimation:
echo   python RunModel.py
echo.