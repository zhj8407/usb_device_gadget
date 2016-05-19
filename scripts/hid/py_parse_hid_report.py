import sys
import re

# Remove the space and comments


def trim_line(raw_line):
    stripped_line = raw_line.strip()
    if stripped_line.startswith('//'):
        return None
    comments_idx = stripped_line.find("//")
    return stripped_line[0:comments_idx].strip()


def str_to_int_list(data_str):
    data_list = []
    for data in re.split(',| ', data_str):
        if data:
            data_list.append(data)

    return data_list


def parse_hid_report(filename):
    hid_report_data_list = []
    try:
        with open(filename, "r") as f:
            for raw_line in f.readlines():
                data_str = trim_line(raw_line)
                if not data_str:
                    continue
                hid_report_data_list.extend(str_to_int_list(data_str))
    except:
        print("Can not open the hid report file " + filename)

    return hid_report_data_list


def write_hid_report(report, filename):
    try:
        with open(filename, 'wb') as f:
            f.write(bytearray(int(x, 0) for x in report))
    except:
        print("Can not write the hid report data to " + filename)


def main():
    argc = len(sys.argv)

    if argc < 3:
        print("Invalid Parameters")
        return -1

    hid_report = parse_hid_report(sys.argv[1])
    write_hid_report(hid_report, sys.argv[2])

    return 0

if __name__ == "__main__":
    main()
