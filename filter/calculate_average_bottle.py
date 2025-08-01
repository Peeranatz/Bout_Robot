import csv

def calculate_average_distance(file_path):
    with open(file_path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # ข้าม header
        distances = [float(row[1]) for row in csv_reader]
        average_distance = sum(distances) / len(distances) if distances else 0
        return average_distance

if __name__ == '__main__':
    file_path = "c:\\Users\\Asus 444\\Desktop\\boutRobot\\tof_bottle.csv"
    average_distance = calculate_average_distance(file_path)
    print(f"Average Distance: {average_distance:.2f} cm")
