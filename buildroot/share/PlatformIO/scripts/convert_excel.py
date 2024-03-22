#
# convert_excel.py
# Converts values in an xls file to _.ini files to be read
#

# add imported modules
try: 
  # necessary dependency was already added
  import xlrd
except: 
  # Because PlatformIO uses its own python environment, must add dependency
  Import("env")
  env.Execute("$PYTHONEXE -m pip install xlrd")
  import xlrd
from pathlib import Path

verbose = 0
def sheet_path(fname):
  return Path("Marlin", fname) 

def config_path(fname):
  return Path("Marlin/configs", fname)

def excel_file(path):
  return xlrd.open_workbook(path)

def process_sheet(workbook):
  worksheet = workbook.sheet_by_index(0)
  first_row = [] # The row where we stock the name of the column
  for col in range(worksheet.ncols):
    first_row.append( worksheet.cell_value(0,col) )
  # transform the workbook to a list of dictionary items
  data =[]
  for row in range(1, worksheet.nrows):
    elm = {}
    for col in range(worksheet.ncols):
        elm[first_row[col]]=worksheet.cell_value(row,col)
    data.append(elm)
  
  # make a dictionary of printers with the name as the key and empty dict as value
  printers = {}
  for printer in first_row[7::]:
    printers[printer] = {}
  
  # populate the settings for each printer
  for param in data:
    for printer in printers:
      # get the printer's dict and set category as key with empty list as value (or do nothing if key exists)
      printers[printer].setdefault(param["Category"],[])
      # append to the list for that category key. Appends the parameter name and the value for that printer
      printers[printer][param["Category"]].append(param["Parameter"]+"="+str(param[printer])+"\n")
  
  # create an ini file for each printer
  for printer in printers:
    printer_map_ini(printer, printers[printer])
  
# turns a map of printer parameters into an ini file
def printer_map_ini(printer_name, printer_params):
  fname = config_path(printer_name+"_config.ini")
  file = open(fname,"w")
  for category in printer_params:
    file.write("[config:"+category+"]\n")
    file.writelines(printer_params[category])
  file.close()
      
if __name__ == "__main__":
    #
    # From command line use the given file name
    #
    import sys
    args = sys.argv[1:]
    if len(args) > 0:
        if args[0].endswith('.xls'):
            sheet_file = args[0]
        else:
            print("Usage: %s <.xls file>" % sys.argv[0])
    else:
        sheet_file = sheet_path('config_sheet.xls')

    sheet_ref = excel_file(sheet_file)
    if sheet_ref:
      process_sheet(sheet_ref)
else:
    #
    # From within PlatformIO use the default value
    #
    sheet_file = sheet_path('config_sheet.xls')
    sheet_ref = excel_file(sheet_file)
    
    if sheet_ref:
      process_sheet(sheet_ref)