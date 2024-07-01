from modules.employee import pyemployee
from modules.employer import pyemployer
from pybind11_abseil import status

if __name__ == '__main__':
    employer = pyemployer.Employer()
    name = 'A'
    id = employer.add_employee(name)
    print(f'Employee {id}\'s name is {employer.get_employee(id).name}')
    print(f'Employee {name}\'s id is {employer.get_employee(name).id}')

    try:
      employer.get_employee(0)
    except status.StatusNotOk as e:
      print(e.status)

    try:
      employer.get_employee('B')
    except status.StatusNotOk as e:
      print(e.status)
