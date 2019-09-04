import csv
in_path = './raw_locations.txt'
out_path = './locations.txt'
in_file = open(in_path, 'r', encoding='utf-8')
out_file = open(out_path, 'w', encoding='utf-8')
rdr = csv.reader(in_file)
data = next(rdr)
for i in range(len(data) // 3):
    out_file.write("['" + data[3*i] + "' , "  + str(float(data[3*i+1])) + ' , ' + str(float(data[3*i+2])) + '], \n')
in_file.close()
out_file.close()
