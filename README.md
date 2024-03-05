# UR Environment



# Environment Setup Instructions

Before running this environment, ensure that the following requirements are met:

## 1. Create a Conda Environment

It's recommended to create a Conda environment with Python 3.11. 

## 2. Clone and Install Python URX Package

- Clone the Python URX repository from GitHub:
  ```
  git clone https://github.com/UoA-CARES/python-urx.git
  ```
- Navigate into the cloned repository directory:
  ```
  cd python-urx
  ```
- Install the Python URX package in editable mode using pip:
  ```
  pip install -e .
  ```

## 3. Install Math3d Package

Install the `math3d` package version 3.4.1:

```
pip install math3d==3.4.1
```

## 4. Test the Environment

Run the provided `general_test.py` example to ensure everything is set up correctly:

```
python general_test.py
```

---

