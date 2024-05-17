from urdf_parser_py.urdf import URDF

kitchen = URDF.from_parameter_server('iai_kitchen')
print (kitchen)
f = open("../../resources/suturo_lab_version_door_open_9.urdf", "w")
f.write(kitchen.to_xml_string())


def urdf_to_string(urdf_file_path):
    try:
        with open(urdf_file_path, 'r') as file:
            urdf_string = file.read()
        return urdf_string
    except FileNotFoundError:
        print(f"The file {urdf_file_path} was not found.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None
