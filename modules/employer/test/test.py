from modules.employee import pyemployee
from modules.employer import pyemployer

if __name__ == '__main__':
    employer = pyemployer.Employer()
    name = 'A'
    id = employer.add_employee(name)
    print(f'Employee {id}\'s name is {employer.get_employee(id).name}')
    print(f'Employee {name}\'s id is {employer.get_employee(name).id}')
