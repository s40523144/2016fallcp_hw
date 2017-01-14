data2 = open("w6_group.txt").read()
#print(data2)
group = data2.splitlines()
#print(group)
#print(group[0])
result_g = []
for i in group:
    #print(i)
    #print(i.split(",")[:-1])
    for j in i.split(",")[:-1]:
        result_g .append(j)
#print(result_g)

data3 = open("w6_list.txt").read()
registered = data3.splitlines()
#print(registered )

for m in registered:
    if m not in result_g:
        print(m)
