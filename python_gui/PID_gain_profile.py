import json
import os

class PID:
    def __init__(self):
        self.Kp = [False, 0]
        self.Kp_div = [False, 0]
        self.Ki = [False, 0]
        self.Ki_div = [False, 0]
        self.Kd = [False, 0]
        self.Kd_div = [False, 0]


class PIDProfileManager:
    def __init__(self):
        self.filename = 'pid_profiles.json'
        self.load_pids()
        
    def load_pids(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    data = json.load(f)
                    self.pids = [PID.from_dict(pid_data) for pid_data in data]
            except json.JSONDecodeError:
                print(f"Error: {self.filename} is not a valid JSON file. Initializing empty PID list.")
                self.pids = []
                self.save_pids()  # 빈 리스트를 JSON 파일로 저장
        else:
            self.pids = []
            self.save_pids()  # 빈 리스트를 JSON 파일로 저장
            
    def save_pids(self):
        with open(self.filename, 'w') as f:
            json.dump([pid.to_dict() for pid in self.pids], f, indent=4)

    def add_profile(self, pid):
        self.pids.append(pid)
        self.save_pids()

    def delete_profile(self, profile_name):
        self.profiles = [profile for profile in self.profiles if profile['name'] != profile_name]

    def display_pids(self):
        """ 모든 PID 객체 출력 """
        if not self.pids:
            print("No PID profiles available.")
            return

        for i, pid in enumerate(self.pids):
            print(f"PID {i + 1}:")
            print(f"  Kp: {pid.Kp}")
            print(f"  Kp_div: {pid.Kp_div}")
            print(f"  Ki: {pid.Ki}")
            print(f"  Ki_div: {pid.Ki_div}")
            print(f"  Kd: {pid.Kd}")
            print(f"  Kd_div: {pid.Kd_div}")
            print()

def main():
    manager = PIDProfileManager()

    while True:
        print("\n--- PID Manager ---")
        print("1. Add PID")
        print("2. Display PIDs")
        print("3. Exit")
        choice = input("Choose an option (1-3): ")

        if choice == '1':
            pid = PID()
            pid.Kp = [True, int(input("Enter Kp (int): "))]
            pid.Kp_div = [True, int(input("Enter Kp_div (int): "))]
            pid.Ki = [True, int(input("Enter Ki (int): "))]
            pid.Ki_div = [True, int(input("Enter Ki_div (int): "))]
            pid.Kd = [True, int(input("Enter Kd (int): "))]
            pid.Kd_div = [True, int(input("Enter Kd_div (int): "))]
            manager.add_pid(pid)

        elif choice == '2':
            manager.display_pids()

        elif choice == '3':
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please try again.")

if __name__ == '__main__':
    main()