
import csv


class DataLogger:
    def __init__(self, log_file_name, field_names, log_data):

        self.log_data = log_data
        # Open and initialize the csv log file
        if self.log_data:
            self.f = open(log_file_name, "w", newline='')
            self.writer = csv.writer(self.f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            # Write the header
            self.writer.writerow(field_names)

    def write_data(self, data_row):
        # write the data row
        if self.log_data:
            self.writer.writerow(data_row)

    def close(self):
        if self.log_data:
            self.f.close()

