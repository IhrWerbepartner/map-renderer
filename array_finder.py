import json


def find_arrays_of_length(data, target_length):
    """
    Recursively searches through nested dictionaries and lists 
    to find and yield arrays of a specific length.
    """
    results = []
    
    # If the current element is a list (array)
    if isinstance(data, list):
        if len(data) == target_length:
            results.append(data)
        # Continue searching inside the elements of the list
        for item in data:
            results.extend(find_arrays_of_length(item, target_length))
            
    # If the current element is a dictionary, search its values
    elif isinstance(data, dict):
        for value in data.values():
            results.extend(find_arrays_of_length(value, target_length))
            
    return results

def process_json_file(file_path, target_length):
    try:
        # Load the JSON file
        with open(file_path, 'r', encoding='utf-8') as file:
            json_data = json.load(file)
            
        # Find matching arrays
        matching_arrays = find_arrays_of_length(json_data, target_length)
        
        # Print results
        print(f"Found {len(matching_arrays)} array(s) of length {target_length}:\n")
        for i, arr in enumerate(matching_arrays, 1):
            print(f"Match {i}: {arr}")
            
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except json.JSONDecodeError:
        print("Error: Failed to decode JSON from the file. Please check the file format.")

# --- Example Usage ---
if __name__ == "__main__":
    # Specify your JSON file path and the target array length (n) here
    file_name = "./samples/streets.json"
    n = 3
    
    process_json_file(file_name, n)
