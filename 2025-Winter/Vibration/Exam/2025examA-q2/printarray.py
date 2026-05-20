def printarray(arr,format1,**kwargs):
    '''
    Function to format and print a 1D array.
      arr: the array
      format1: a string with the required format, e.g., .5e for exponential with 5 decimal digits
      kwargs: name: a string with the name of the array in the calling program, to be prefixed
              eu: engineeering units, to be postfixed
    '''

    # print prefix if any
    if 'name' in kwargs:
        print(kwargs['name'],' = [',end="")
    else:
        print('[',end="")
    
    # print array
    nn = arr.size
    fm = '{:'+format1+'} '
    for ii in range(nn):print(fm.format(arr[ii]),end="")
    print(']',end="")
    
    # print units if any
    if 'eu' in kwargs:
        print(' ',kwargs['eu'])
    else:
        print('')
        
    return