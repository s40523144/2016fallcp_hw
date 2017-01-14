data = open("midterm_list.txt").read()
line_list = data.splitlines()
count = 0
for line in line_list:
    each_line_list = line.split("\t")
    try:
        if "Obsoleted" in each_line_list[5]:
            count = count + 1
            print(each_line_list[0])
    except:
        pass
print(count)