from rclpy.node import Node

class CommonRosNode(Node):
    def __init__(self):
        # Initialize the Core node with a default name, allowing undeclared parameters.
        super().__init__(
            "default_name",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Load parameters from YAML configurations.
        self.__localParameters = self.load_parameters()

        # universal parameter
        self.RATE_PARAM_NAME = "rate"
        self.rate = self.get_rate()

    def load_parameters(self):
        """Load all parameters from the node and store them in a nested dictionary.

        This function retrieves all parameters from the protected attribute `_parameters`
        (inherited from `Node`) and organizes them in a structured dictionary.
        
        Returns:
            dict: A nested dictionary containing all parameters and their values.
        
        Notes:
            Since there is no official way to list all parameters, `_parameters` is accessed directly.
            If a better approach exists, suggestions are welcome.
        """
        parameters = {}

        # Iterate over all available parameters in the node.
        for param_name in self._parameters:
            try:
                param_value = self.get_parameter(param_name).get_parameter_value()

                # Split the parameter name into parts using dots as delimiters
                parts = param_name.split(".")
                current_dict = parameters

                # Iterate through all parts of the name to rebuild the nested structure  
                for part in parts[:-1]:
                    if part not in current_dict:
                        current_dict[part] = {}
                    current_dict = current_dict[part]

                # Assign the value to the last level of the structure  
                last_part = parts[-1]

                # Match the parameter type to assign the correct value.
                match param_value.type:
                    case 1:
                        current_dict[last_part] = param_value.bool_value
                    case 2:
                        current_dict[last_part] = param_value.integer_value
                    case 3:
                        current_dict[last_part] = param_value.double_value
                    case 4:
                        current_dict[last_part] = param_value.string_value
                    case 5:
                        current_dict[last_part] = param_value.byte_array_value
                    case 6:
                        current_dict[last_part] = param_value.bool_array_value
                    case 7:
                        current_dict[last_part] = param_value.integer_array_value
                    case 8:
                        current_dict[last_part] = param_value.double_array_value
                    case 9:
                        current_dict[last_part] = param_value.string_array_value
                    case _:
                        raise TypeError(f"Unknown type for parameter: {param_name}")
            except Exception as e:
                self.get_logger().error(f"Error loading parameter {param_name}: {str(e)}")
        return parameters

    def get_parameter_value(self, param_name, default=None):
        """Retrieve a parameter from the local parameter dictionary.

        Args:
            param_name (str): The name of the parameter to retrieve.
            default (Any, optional): The default value to return if the parameter is not found. Defaults to None.

        Returns:
            Any: The parameter value if found, otherwise the default value.
        """
        if param_name in self.__localParameters:
            return self.__localParameters[param_name]
        else:
            self.get_logger().warn(f"Parameter '{param_name}' not found, using default: {default}")
            return default


    # Function to get the rate parameter.
    def get_rate(self):
        return self.get_parameter_value(self.RATE_PARAM_NAME, default=30.0)
