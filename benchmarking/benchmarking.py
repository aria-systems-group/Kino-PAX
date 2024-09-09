import csv
import numpy as np

def calculate_execution_time(file_path):
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        data = [float(row[0]) for row in reader]
    
    mean = np.mean(data)
    std_dev = np.std(data)
    min_val = np.min(data)
    max_val = np.max(data)
    
    return mean, std_dev, min_val, max_val

def main():
    file_path = 'build/Data/ExecutionTime/executionTime.csv'
    mean, std_dev, min_val, max_val = calculate_execution_time(file_path)
    

    print("/* Execution Time */")
    print(f"Mean : {mean}")
    print(f"Standard Deviation: {std_dev}")
    print(f"Minimum: {min_val}")
    print(f"Maximum: {max_val}")
    print("/***************************/\n")

if __name__ == "__main__":
    main()