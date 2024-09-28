#!/usr/bin/env python3

import argparse
import os
import sys

import pandas as pd
from matplotlib import pyplot as plt

def main(filename):
    # Print out the filename if it exists
    print(f"Processing file: {filename}")

    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    df = pd.read_csv(filename)

    plt.plot(df['time'], df['z'])
    plt.show(block='True')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a file.")
    
    # Add argument for filename, type=str is implicit
    parser.add_argument("filename", help="The name of the file to process")
    
    # Parse arguments
    args = parser.parse_args()

    # Check if the file exists
    if not os.path.isfile(args.filename):
        print(f"Error: The file '{args.filename}' does not exist.", file=sys.stderr)
        sys.exit(1)  # Exit with a non-zero status for error
    
    # If the file exists, call main
    main(args.filename)