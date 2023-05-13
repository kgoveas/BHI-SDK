import fnmatch,os,sys
import argparse
from argparse import RawTextHelpFormatter
from itertools import count, filterfalse # ifilterfalse on py2

description_text='BHy2 SDK utility to deal with driver IDs. Functions are:\n \
              a) print all drivers contained in the SDK (no argument) \n \
              b) search all drivers in the SDK by ID    (-f) \n \
              c) search all drivers in the SDK by name  (-s) \n \
              d) look up available IDs for new drivers  (--id) '
parser = argparse.ArgumentParser(description=description_text, formatter_class=RawTextHelpFormatter)
group = parser.add_mutually_exclusive_group()
group.add_argument('--id', action='store_true', dest='print_available_ids', help='print out the next available driver IDs for the current SDK')
group.add_argument('-f', type=int, dest='search_id',   metavar='ID',   help='find the name for a driver by a given driver ID')
group.add_argument('-s', type=str, dest='search_name', metavar='name', help='search for a driver ID by a given driver name')
parser.add_argument('--version', action='version', version='%(prog)s v1.0')
args = parser.parse_args()

#print ('Config script reading out driver IDs from the SDK (give ID as parameter to search)')
#print

driver_id_max=0xFF
driver_id_min = 0x01
phys_driver_min = driver_id_min
phys_driver_max = 0x3F
virt_driver_min = phys_driver_max+1
virt_driver_max = driver_id_max

cmakelist_id_string = 'SET(DRIVER_ID'
cmakelist_imported_string = 'ADD_IMPORTED_DRIVER('

ids = []
directories=['drivers','drivers_custom']
patterns=['CMakeLists.txt','*.sdk.cmake'] # include 

for directory in directories:
    for root, dirnames, filenames in os.walk(directory):
        for p in patterns:
            for filename in fnmatch.filter(filenames, p):
                if(root == directory): #exclude the root CMakeLists.txt of the drivers folder
                    continue
                if(root == os.path.join(directory, 'VirtHangDetection') and filename == patterns[0]): # exclude CMakeLists.txt of VirtHangDetection, which has separate .cmake file
                    continue
                id_count=len(ids) 
                with open(os.path.join(root, filename)) as f:   # open file
                    for line in f:       # process line by line
                        if cmakelist_id_string in line:    # search for string
                            #path=root.split('/') # likely not to work in windows
                            #driver_name=path[-1] # 
                            id_string=line.split(' ') # split by the space before the ID
                            driver_id=id_string[1]
                            driver_id = int(driver_id[:-2]) #remove trailing bracket and newline
                            #entry = [driver_id, root, driver_name]
                            entry = [driver_id, root]
                            ids.append(entry)
                            break
                        elif cmakelist_imported_string in line:    # search for imported precompiled drivers
                            id_string = line.split(' ') # split by the space before the ID
                            driver_id = id_string[1]
                            #(no_use, driver_name) = id_string[0].split('(') # split the driver name from the cmake function call
                            #entry = [driver_id, root, driver_name]
                            entry = [int(driver_id), root]
                            ids.append(entry)
                            break
                if(id_count==len(ids)):
                    print ('Error, found CMakeLists.txt without Driver ID : %s' %os.path.join(root, filename))

ids = sorted(ids)

# calling with ID or name  to search for it
if((args.search_id is not None) or (args.search_name is not None)):
    if(args.search_id is not None):
        if(args.search_id > driver_id_max):
            print ('Driver IDs have to be in the range from %i-%i' %(driver_id_min, driver_id_max))
            sys.exit(1)
        print ('Searching drivers for ID %i:' %args.search_id)
        # Search the list of IDs
        result_idxs = []
        for index, id in enumerate(ids): # iterate the sorted list of ids with an index
            if id[0] == args.search_id:
                # Display the found list entry
                print('ID: %i in %s' %(id[0], id[1]))
                result_idxs.append(index)
                
        if len(result_idxs) == 0:
            print ('ID %i not found in drivers and drivers_custom folders' %args.search_id)
            print
        elif len(result_idxs) > 1:
            print('')
            print ('Warning! Multiple driver IDs found')
    else:
        print ('Searching ID for driver %s:' %args.search_name)
        # Search the list of IDs
        result_idxs = []
        for index, id in enumerate(ids): # iterate the sorted list of ids with an index
            if os.path.basename(os.path.normpath(id[1])) == args.search_name:
                # Display the found list entry
                print('ID: %i in %s' %(id[0], id[1]))
                result_idxs.append(index)
                
        if len(result_idxs) == 0:
            print ('Driver %s not found in drivers and drivers_custom folders' %args.search_name)
            print
        elif len(result_idxs) > 1:
            print('')
            print ('Warning! Multiple driver IDs found')


# user wants to receive a list of available driver IDs
elif (args.print_available_ids is True):
    print ('Lowest available driver IDs in this SDK are:')

    id_set = [item[0] for item in ids]
    filter = filterfalse(set(id_set).__contains__, count(phys_driver_min))
    phys_id_one = next(filter)
    phys_id_two = next(filter)
    phys_id_three = next(filter)

    filter = filterfalse(set(id_set).__contains__, count(virt_driver_min))
    virt_id_one   = next(filter)
    virt_id_two   = next(filter)
    virt_id_three = next(filter)

    if (phys_id_one > phys_driver_max):
        print('No available IDs for physical drivers, please remove some unused drivers!')
    elif (phys_id_two > phys_driver_max):
        print('Only one for physical drivers: %i;' %(phys_id_one))
    elif (phys_id_three > phys_driver_max):
        print('Only two for physical drivers: %i; %i;' %(phys_id_one, phys_id_two))
    else:
        print('Physical drivers: %i; %i; %i' %(phys_id_one, phys_id_two, phys_id_three))

    if (virt_id_one > virt_driver_max):
        print('No available IDs for virtual drivers, please remove some unused drivers!')
    elif (virt_id_two > virt_driver_max):
        print('Only one for virtual drivers: %i;' %(virt_id_one))
    elif (virt_id_three > virt_driver_max):
        print('Only two for virtual drivers: %i; %i;' %(virt_id_one, virt_id_two))
    else:
        print('Virtual drivers: %i; %i; %i' %(virt_id_one, virt_id_two, virt_id_three))

# no search, no inquiry listing all IDs
else:
    id_out_of_bounds = False
    print ('Driver IDs found in the SDK (found %i drivers):' %len(ids))
    for id in ids: # print all ids one by one
        if((id[0] < driver_id_min) or (id[0] > driver_id_max)): # check if found id is within boundaries
            id_out_of_bounds = True
            print('')
            print('Warning, driver ID out of range found:')
            print('ID: %i in %s' %(id[0], id[1]))
            print('')
        else:
            print('ID: %i in %s' %(id[0], id[1]))

    # Check for duplicates
    seen = set()
    dup_idxs = []
    for index, id in enumerate(ids): # iterate the sorted list of ids with an index
        if id[0] not in seen:
            seen.add(id[0])
        else:
            i = index
            # find all the previous occurrences of this particular ID
            # This may be reached multiple times for an ID that is duplicated several times, and each time append all previous duplicates
            while(ids[i][0] == id[0]):
                dup_idxs.append(i)
                i -= 1

    # Warn if duplicates have been found and print a list of them
    if len(dup_idxs)>0:
        print('')
        print ('Warning! Duplicate driver IDs found:')
        # remove duplicates in list of duplicates due to the multiple calling above
        dup_idxs = sorted(list(set(dup_idxs)))
        for index in dup_idxs:
            print('ID: %i in %s' %(ids[index][0], ids[index][1]))

    if(id_out_of_bounds is True):
        print('\nWarning, driver ID out of range found! Please check the list above')
