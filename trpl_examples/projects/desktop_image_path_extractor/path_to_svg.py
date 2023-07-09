def convert_paths_to_svg(paths, file_path):
    svg_content = '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
    svg_content += '<svg xmlns="http://www.w3.org/2000/svg" version="1.1">\n'
    
    for path in paths:
        path_string = 'M ' + ' '.join([f"{point[0]},{point[1]}" for point in path])
        svg_content += f'\t<path d="{path_string}" fill="none" stroke="black" />\n'
    
    svg_content += '</svg>'
    
    with open(file_path, 'w') as svg_file:
        svg_file.write(svg_content)
    
    print(f'SVG file saved to: {file_path}')

# Example usage
# paths = [
#     [[100, 100], [200, 200], [300, 100]],
#     [[50, 50], [100, 100], [150, 50], [100, 0]],
# ]

# convert_paths_to_svg(paths, 'output.svg')
