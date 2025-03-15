import yaml
import rclpy

def read_pid_yaml_and_generate_parameters(node_name, file_path):
    """
    Reads a PID configuration YAML file and generates a list of tuples
    with parameter names and rclpy.Parameter.Type.DOUBLE, excluding config_location.

    Args:
        file_path (str): Path to the YAML file.

    Returns:
        List[Tuple[str, rclpy.Parameter.Type]]: List of tuples containing parameter names
                                                and types.
    """
    parameters = []

    try:
        # Open and parse the YAML file
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        pid_parameters = data.get(node_name, {}).get('ros__parameters', {})

        # Iterate through the parameters and exclude 'config_location'
        for param_name, param_value in pid_parameters.items():
            if param_name != 'config_location':
                if 'location' in param_name:
                    parameters.append((param_name, rclpy.Parameter.Type.STRING))
                else:
                    parameters.append((param_name, rclpy.Parameter.Type.DOUBLE))


    except Exception as e:
        print(f"Error reading or parsing YAML file: {e}")

    return parameters

# Example usage
if __name__ == "__main__":
    # Replace with the actual path to your YAML file
    yaml_file_path = "/path/to/pid.yaml"
    parameter_list = read_pid_yaml_and_generate_parameters(yaml_file_path)
    print(parameter_list)
