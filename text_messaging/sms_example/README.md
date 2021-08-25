# Text Messaging Example

This program shows an example of text messaging from inside a robot program.
The program executes a simple move three times, sending a text to the destination phone number each time it completes the move.

This example is discussed on the Tormach user forums here: 

## Prerequisites
To run this example code you will need to sign up for an account with Twilio.  New accounts get a $10 credit, no CC information required as of 8/2021.  You must get from Twilio the following information:
1. Account SID
2. Authorization Token
3. "From" phone number

Then fill in that information in the following fields in sms.py:
account_sid = ''
auth_token = ''
from_number=''

In sms_example.py, change the destination phone number to the number you want to text.

NB: all phone numbers should be formatted like this:
'+6085551212'

Lack of a leading + sign, spaces, parentheses, or hyphens will not work.

## Libraries Used
Twilio: https://www.twilio.com/docs/libraries/python
