# Example 13: Using get_param() and set_param()
This program will demonstrate how to store and manage data using get_param()/set_param().
Data stored using set_param() will be persistent, meaning you can still access the data even after stopping and starting the program. 
Also the data can be accessed between programs. 

To store data we call set_param() which takes in two arguments.
* set_param(name, value):
    name: 
        A string that will be used as an access key for the data you store.
    value: 
        The data you want to store. It can be any python or RPL data type

To get the stored data we call get_param() which also takes in two arguments but the second argument is optional.
* get_param(name, default=None):
    name: 
        A string access key that the data was store under.
    default: 
        The default value that will be returned if nothing was  found. By default "None" is returned as the default value.
To delete the saved data, we use delete_param() which takes in one argument.
* delete_param(name):
    name: The string access key that the data was saved under.

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language