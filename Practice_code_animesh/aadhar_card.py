class License:
    def __init__(self,Name,Address,DOB):
        self.Name = Name
        self.Address = Address
        self.DOB = DOB

person1 = License("Animesh","C-1/102","2nd May")
person2 = License("Charudatta","C-1/103","23rd Sept")
print(person1.Address)

