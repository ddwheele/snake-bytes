#!/usr/bin/env python3

import argparse
import os
import sys

import pandas as pd
from matplotlib import pyplot as plt

def main(filename, variable_names):
    # Print out the filename if it exists
    print(f"Processing file: {filename}")

    # Set up matplotlib parameters
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    
    # Read the CSV file into a DataFrame
    df = pd.read_csv(filename)

    # Loop through each variable name provided in the command line arguments
    for variable in variable_names:
        if variable in df.columns:
            print("plotting time vs " + variable)
            plt.plot(df['time'], df[variable], label=variable)
        else:
            print(f"Warning: '{variable}' is not a valid column in the CSV file.", file=sys.stderr)

    # Add labels and a legend
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Time vs Variables')
    plt.legend()  # Show a legend to differentiate the variables

    plt.grid(True) 
    
    # Show the plot
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot variables from a file.")
    
    # Add argument for filename
    parser.add_argument("filename", help="The name of the file to process")
    # Use 'nargs' to accept a variable number of variable names
    parser.add_argument("variables", nargs='+', help="List of variable names to plot")

    # Parse arguments
    args = parser.parse_args()

    # Check if the file exists
    if not os.path.isfile(args.filename):
        print(f"Error: The file '{args.filename}' does not exist.", file=sys.stderr)
        sys.exit(1)  # Exit with a non-zero status for error
    
    # Call the main function with the filename and list of variable names
    main(args.filename, args.variables)