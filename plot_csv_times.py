#!/usr/bin/env python3

import argparse
import os
import sys

import pandas as pd
from matplotlib import pyplot as plt


def main(filename, variable_names, time_a, time_b):
    # Print out the filename if it exists
    print(f"Processing file: {filename}")

    # Set up matplotlib parameters
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    
    # Read the CSV file into a DataFrame
    df = pd.read_csv(filename)

    # Ensure 'time' is in the correct format (if needed, depending on your data)
    df['time'] = pd.to_numeric(df['time'], errors='coerce')  # Convert to numeric

    # Filter the DataFrame based on the specified time range
    filtered_df = df[(df['time'] >= time_a) & (df['time'] <= time_b)]

    # Loop through each variable name provided in the command line arguments
    for variable in variable_names:
        if variable in filtered_df.columns:
            plt.plot(filtered_df['time'], filtered_df[variable], label=variable)
        else:
            print(f"Warning: '{variable}' is not a valid column in the CSV file.", file=sys.stderr)

    # Add labels and a legend
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title(f'Time vs Variables from {time_a} to {time_b}')
    plt.legend()  # Show a legend to differentiate the variables

    # Add grid lines
    plt.grid(True)  # You can customize the appearance of the grid lines here

    # Show the plot
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot variables from a file.")
    
    # Add argument for filename
    parser.add_argument("filename", help="The name of the file to process")
    # Use 'nargs' to accept a variable number of variable names
    parser.add_argument("variables", nargs='+', help="List of variable names to plot")
    # Add arguments for the time range
    parser.add_argument("time_a", type=float, help="Start time for the plot")
    parser.add_argument("time_b", type=float, help="End time for the plot")

    # Parse arguments
    args = parser.parse_args()

    # Check if the file exists
    if not os.path.isfile(args.filename):
        print(f"Error: The file '{args.filename}' does not exist.", file=sys.stderr)
        sys.exit(1)  # Exit with a non-zero status for error
    
    # Call the main function with the filename, list of variable names, and time range
    main(args.filename, args.variables, args.time_a, args.time_b)