import os
import re

def list_components(base_dir):
    """Lists valid component directories within the base directory."""
    components = []
    if not os.path.isdir(base_dir):
        return components
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        # Check if it's a directory and contains Source and Include subdirs
        if os.path.isdir(item_path) and \
           os.path.isdir(os.path.join(item_path, 'Source')) and \
           os.path.isdir(os.path.join(item_path, 'Include')):
            components.append(item)
    return components

def main():
    """Generates a .c/.h file pair within an existing ESP-IDF component."""

    script_dir = os.path.dirname(__file__)
    template_dir = script_dir
    components_base_dir = os.path.abspath(os.path.join(script_dir, '../../Application/components'))

    print("ESP-IDF Source File Pair Generator")
    print("----------------------------------")

    # --- Get User Input: File Pair Name ---
    while True:
        file_base_name = input("Enter the new .c/.h file pair base name (e.g., new_driver): ").strip()
        if file_base_name and re.match(r'^[a-zA-Z0-9_]+$', file_base_name):
            break
        else:
            print("Invalid file name. Use letters, numbers, and underscores only.")

    # --- Get User Input: Component Selection ---
    available_components = list_components(components_base_dir)
    if not available_components:
        print(f"Error: No components found in '{components_base_dir}'.")
        print("Please ensure components exist with 'Source' and 'Include' subdirectories.")
        return

    print("\nAvailable components:")
    for i, comp in enumerate(available_components):
        print(f"  {i + 1}. {comp}")

    while True:
        try:
            choice = input(f"Select the component number (1-{len(available_components)}): ").strip()
            choice_index = int(choice) - 1
            if 0 <= choice_index < len(available_components):
                selected_component_name = available_components[choice_index]
                break
            else:
                print("Invalid selection.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    # --- Define Paths ---
    component_path = os.path.join(components_base_dir, selected_component_name)
    source_path = os.path.join(component_path, 'Source')
    include_path = os.path.join(component_path, 'Include')
    c_template_path = os.path.join(template_dir, 'template.c')
    h_template_path = os.path.join(template_dir, 'template.h')
    output_c_path = os.path.join(source_path, f'{file_base_name}.c')
    output_h_path = os.path.join(include_path, f'{file_base_name}.h')

    # --- Check for Existing Files ---
    if os.path.exists(output_c_path):
        print(f"Error: Source file '{output_c_path}' already exists.")
        return
    if os.path.exists(output_h_path):
        print(f"Error: Include file '{output_h_path}' already exists.")
        return

    # --- Generate .c and .h files ---
    try:
        with open(c_template_path, 'r') as f:
            c_template_content = f.read()
        with open(h_template_path, 'r') as f:
            h_template_content = f.read()

        component_name_upper = selected_component_name.upper()
        file_name_upper = file_base_name.upper()

        # Generate .c file
        c_content = c_template_content
        c_content = c_content.replace('@file template.c', f'@file {file_base_name}.c')
        c_content = c_content.replace('#include "Common.h"', f'#include "{file_base_name}.h"') # Include corresponding .h
        c_content = c_content.replace('#define TAG "TEMPLATE"', f'#define TAG "{component_name_upper}"')
        c_content = c_content.replace('template_function', f'{file_base_name}_function') # Example static function

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

        with open(output_h_path, 'w') as f:
            f.write(h_content)
        print(f"Generated file: {output_h_path}")

        print(f"\nSource file pair '{file_base_name}.c'/'{file_base_name}.h' generated in component '{selected_component_name}'.")
        # Make the reminder more visible
        print("\n" + "="*60)
        print("IMPORTANT:")
        print(f"Remember to add 'Source/{file_base_name}.c' to the SRCS list in:")
        print(f"  {component_path}/CMakeLists.txt")
        print("="*60 + "\n")

    except IOError as e:
        print(f"Error processing template files: {e}")
        return
    except OSError as e:
        print(f"Error writing output files: {e}")
        return

if __name__ == "__main__":
    main()