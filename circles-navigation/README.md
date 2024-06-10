# Folder Structure
1. `data/`: Stores various datasets and information crucial for this task.
     - `data/external`: Contains datasets and information sourced from external sources, such as publicly available datasets, third-party providers, or other research projects. These datasets may include images, videos, sensor data, or other types of input data used for training and testing AI models.
     - `data/interim`: Serves as an intermediate stage in the data preprocessing pipeline. It typically contains partially processed or temporary data files generated during the data preparation phase.
     - `data/processed`: Stores the finalized and processed datasets ready for training AI models.
1. `images/`: Contains the images used in this directory, including those utilized for making predictions.
1. `models/`: Stores models used for AI training in this task.
   - `models/best-models`: stores the best-performing models selected from the trained models based on certain criteria, such as performance metrics like accuracy, precision, or loss.
   - `models/trained-models`: Contains all trained models, including intermediate models generated during the training process.  
1. `notebooks/`: Contains Jupyter notebooks used for various purposes within this task.
1. `results/`: Stores the output, findings, and outcomes generated from various analyses, experiments, or simulations conducted as part of this task. 
1. `src/`: Serves as the main source code folder for this task.
     - `src/models`: Contains scripts to train models and then use trained models to make predictions.
     - `src/visualization`: Contains scripts to create exploratory and results-oriented visualizations.
     - `src/control`: Contains source code related to the control system of the ROV. It contains scripts, modules, or packages responsible for controlling the various aspects of the ROV's behavior, including movement, navigation, sensor interaction, and communication with external devices or systems.
