import os
import re

def main():
    """Generates an ESP-IDF component structure from templates."""

    script_dir = os.path.dirname(__file__)
    template_dir = script_dir
    output_base_dir = os.path.abspath(os.path.join(script_dir, '../../Application/components'))

    print("ESP-IDF Component Generator")
    print("---------------------------")

    # --- Get User Input ---
    while True:
        component_name = input("Enter the new component name (e.g., my_sensor): ").strip()
        if component_name and re.match(r'^[a-zA-Z0-9_]+$', component_name):
            break
        else:
            print("Invalid component name. Use letters, numbers, and underscores only.")

    while True:
        file_pairs_str = input("Enter .c/.h file pair base names, separated by spaces (e.g., driver utils): ").strip()
        if file_pairs_str:
            file_pairs = file_pairs_str.split()
            if all(re.match(r'^[a-zA-Z0-9_]+$', fp) for fp in file_pairs):
                break
            else:
                print("Invalid file names. Use letters, numbers, and underscores only.")
        else:
            print("Please enter at least one file pair name.")

    dependencies_str = input("Enter additional component dependencies (REQUIRES), separated by spaces (e.g., esp_log nvs_flash) [optional]. (basic dependencies already included: driver esp_common freertos): ").strip()
    dependencies = dependencies_str.split() if dependencies_str else []

    # --- Define Paths ---
    component_path = os.path.join(output_base_dir, component_name)
    source_path = os.path.join(component_path, 'Source')
    include_path = os.path.join(component_path, 'Include')
    cmake_template_path = os.path.join(template_dir, 'CMakeLists.txt')
    c_template_path = os.path.join(template_dir, 'template.c')
    h_template_path = os.path.join(template_dir, 'template.h')

    # --- Check for Existing Component ---
    if os.path.exists(component_path):
        print(f"Error: Component directory '{component_path}' already exists.")
        return

    # --- Create Directories ---
    try:
        os.makedirs(source_path)
        os.makedirs(include_path)
        print(f"Created directory: {component_path}")
        print(f"Created directory: {source_path}")
        print(f"Created directory: {include_path}")
    except OSError as e:
        print(f"Error creating directories: {e}")
        return

    # --- Generate CMakeLists.txt ---
    try:
        with open(cmake_template_path, 'r') as f:
            cmake_content = f.read()

        src_list_str = " ".join([f'"Source/{fp}.c"' for fp in file_pairs])
        deps_list_str = " ".join(dependencies)

        cmake_content = cmake_content.replace("*list_of_source_files*", src_list_str)
        # Ensure there's a space before adding new dependencies if the list is not empty
        if deps_list_str:
             cmake_content = cmake_content.replace("*list_of_additional_dependencies*", "" + deps_list_str)
        else:
             cmake_content = cmake_content.replace("*list_of_additional_dependencies*", "")


        output_cmake_path = os.path.join(component_path, 'CMakeLists.txt')
        with open(output_cmake_path, 'w') as f:
            f.write(cmake_content)
        print(f"Generated file: {output_cmake_path}")

    except IOError as e:
        print(f"Error processing CMakeLists.txt: {e}")
        return

    # --- Generate .c and .h files ---
    try:
        with open(c_template_path, 'r') as f:
            c_template_content = f.read()
        with open(h_template_path, 'r') as f:
            h_template_content = f.read()

        component_name_upper = component_name.upper()

        for file_base_name in file_pairs:
            file_name_upper = file_base_name.upper()

            # Generate .c file
            c_content = c_template_content
            c_content = c_content.replace('@file template.c', f'@file {file_base_name}.c')
            c_content = c_content.replace('#include "Common.h"', f'#include "{file_base_name}.h"') # Include corresponding .h
            c_content = c_content.replace('#define TAG "TEMPLATE"', f'#define TAG "{component_name_upper}"')
            c_content = c_content.replace('template_function', f'{file_base_name}_function') # Example static function

            output_c_path = os.path.join(source_path, f'{file_base_name}.c')
            with open(output_c_path, 'w') as f:
                f.write(c_content)
            print(f"Generated file: {output_c_path}")

            # Generate .h file
            h_content = h_template_content
            h_content = h_content.replace('@file template.h', f'@file {file_base_name}.h')
            h_content = h_content.replace('#ifndef TEMPLATE_H', f'#ifndef {file_name_upper}_H')
            h_content = h_content.replace('#define TEMPLATE_H', f'#define {file_name_upper}_H')
            h_content = h_content.replace('template_perform_action', f'{file_base_name}_perform_action') # Example global function
            h_content = h_content.replace('#endif /* TEMPLATE_H */', f'#endif /* {file_name_upper}_H */')

            output_h_path = os.path.join(include_path, f'{file_base_name}.h')
            with open(output_h_path, 'w') as f:
                f.write(h_content)
            print(f"Generated file: {output_h_path}")

    except IOError as e:
        print(f"Error processing template files: {e}")
        return

    print("\nComponent generation complete!")

if __name__ == "__main__":
    main()