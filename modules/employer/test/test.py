from modules.employee import pyemployee
from modules.employer import pyemployer

if __name__ == '__main__':
    employer = pyemployer.Employer()
    employer.add_employee(0, "A")
    print(f'Employee 0\'s name is {employer.get_employee(0).name}')
    print(f'Employee A\'s id is {employer.get_employee("A").id}')
