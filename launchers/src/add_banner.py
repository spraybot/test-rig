#v1************************************************************************************
# This is developed at Carnegie Mellon University in collaboration with Autel Robotics 
#                                                                                      
# PI:                                                                                  
# George Kantor                                                                        
#                                                                                      
# Authors:                                                                              
# Weizhao Shao                                                                         
# Cong Li                                                                              
# Srinivasan Vijayarangan                                                              
#                                                                                      
# Please refer to the contract document for details on license/copyright information.  
#**************************************************************************************

import sys
import os

ch_banner = """/*v1*************************************************************************************/
/* This is developed at Carnegie Mellon University in collaboration with Autel Robotics */
/*                                                                                      */
/* PI:                                                                                  */
/* George Kantor                                                                        */
/*                                                                                      */
/* Authors:                                                                             */ 
/* Weizhao Shao                                                                         */
/* Cong Li                                                                              */
/* Srinivasan Vijayarangan                                                              */
/*                                                                                      */
/* Please refer to the contract document for details on license/copyright information.  */
/****************************************************************************************/
"""

py_banner = """#v1************************************************************************************
# This is developed at Carnegie Mellon University in collaboration with Autel Robotics 
#                                                                                      
# PI:                                                                                  
# George Kantor                                                                        
#                                                                                      
# Authors:                                                                              
# Weizhao Shao                                                                         
# Cong Li                                                                              
# Srinivasan Vijayarangan                                                              
#                                                                                      
# Please refer to the contract document for details on license/copyright information.  
#**************************************************************************************
"""

xml_banner = """<!-- 
*v1*************************************************************************************
* This is developed at Carnegie Mellon University in collaboration with Autel Robotics *
*                                                                                      *
* PI:                                                                                  *
* George Kantor                                                                        *
*                                                                                      *
* Authors:                                                                             * 
* Weizhao Shao                                                                         *
* Cong Li                                                                              *
* Srinivasan Vijayarangan                                                              *
*                                                                                      *
* Please refer to the contract document for details on license/copyright information.  *
****************************************************************************************
-->
"""

def insert_banner(originalfile,string):
    with open(originalfile,'r') as f:
        with open('tempfile','w') as f2: 
            f2.write(string)
            f2.write(f.read())
    os.rename('tempfile',originalfile)



#recursive function that searches through directories and adds banner
def add_banner_recursive(d):
    files = os.listdir(d)
    for f in files:

        f = os.path.join(d,f)

        if os.path.isdir(f):
            if './' != f  and '.git' not in f:
                print('Stepping into '+f)
                add_banner_recursive(f)
        else:
            #get user consent
            resp = input('Write banner to '+f+'? (y/n): ')

            if 'y' in resp: 
                if '.c' in f or '.h' in f or '.ino' in f:
                    insert_banner(f,ch_banner)
                elif '.py' in f or 'CMakeLists' in f or '.msg' in f or '.yaml' in f or '.sh' in f:
                    insert_banner(f,py_banner)
                elif '.launch' in f :
                    insert_banner(f,xml_banner)
                else:
                    print('Ignoring this file format')

if __name__ == '__main__':

    args = sys.argv

    if len(args) < 2:
        print('Usage: python add_banner.py <dir>')
        sys.exit(-1)

    add_banner_recursive(args[1])
