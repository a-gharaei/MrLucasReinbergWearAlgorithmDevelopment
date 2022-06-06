# MrLucasReinbergWearAlgorithmDevelopment

This project is about developing a macro fracture wear algorithm for a kinematics-geometric grinding simulation

## Installation

1. Clone the fracture_wear repository in your desired directory
2. In the terminal navigate to the project directory

```bash
cd "C:\some_directory\fracture_wear"
```
3. Make a new virtual environment using [pipenv](https://pypi.org/project/pipenv/)

```bash
pipenv shell
```

Then install the required packages

```bash
pipenv install requirements.txt
```

## Usage

Main.py contains a simple example. Just run Main.py and see the macro fracture of a grain!
If you want to include it in a simlation, invoke the function apply_macro_fracture_wear() from the wear_models.py file in your main simulation loop.

