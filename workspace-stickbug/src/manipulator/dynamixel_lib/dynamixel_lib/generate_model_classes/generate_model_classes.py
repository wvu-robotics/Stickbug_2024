from __future__ import annotations
import os, csv 

def parse_line_to_ctrl_table_obj(line: list[str]):
    column_address = 0 
    column_data_name = 2
    column_size_bytes = 1 
    object_class_name = '_CtrlTableValue'

    def ctrl_table_obj_name(line: list[str]):
        replacements = {" ": "",
                        ")": "",
                        "(": ""}
        obj_name = line[column_data_name]
        for old, new in replacements.items():
            obj_name = obj_name.replace(old,new)
        return obj_name

    def valid_line(unchecked_line: list[str]):
        return unchecked_line[0].isdigit()
    
    def get_access_index(line: list[str]):
        for column_num, column_contents in enumerate(line):
            if column_contents.startswith('R') and (column_contents.endswith('R') or column_contents.endswith('W')) :
                return column_num
    
    if valid_line(line):
        return (f"\t{ctrl_table_obj_name(line)} = {object_class_name}({line[column_address]},{line[column_size_bytes]},'{line[get_access_index(line)]}')\n")
    else:
        return ''

def parse_csv_into_model_class(new_class_name: str, file_path: str):
    lines_to_write = []
    lines_to_write.append(f'class {new_class_name}(_DynamixelModel):\n')
    with open(file_path, newline='\n') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for csv_line in csvreader:
            lines_to_write.append(parse_line_to_ctrl_table_obj(csv_line))
    
    lines_to_write.append('\n')
    return lines_to_write


def main():
    csv_directory = 'dynamixel_lib/generate_model_classes/model_ctrl_tables'
    new_file_path = 'dynamixel_lib/dynamixel_models.py'

    with open(new_file_path,'w+') as new_file:

        new_file.write('from dynamixel_lib.generate_model_classes.ctrl_table_value import _CtrlTableValue\n\n')
        new_file.write('class _DynamixelModel():\n')
        new_file.write('\t# This class is to ensure that all created new classes are the same class\n\t# and that type checking can happen correctly in the Motor Class in motor.py\n')
        new_file.write('\tpass\n\n')

        for filename in os.listdir(csv_directory):
            file_path = os.path.join(csv_directory, filename)
            if os.path.isfile(file_path):
                print(f'Parsing {filename} into a model class')
                new_class_name = os.path.splitext(filename)[0]
                new_file.writelines(parse_csv_into_model_class(new_class_name,file_path))
                
                
if __name__ == '__main__':
    main()