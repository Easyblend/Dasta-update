import json
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class JSONFileHandler(FileSystemEventHandler):
    def __init__(self, json_file_path):
        self.json_file_path = json_file_path
        self.last_data = self.read_json()

    def read_json(self):
        try:
            with open(self.json_file_path, 'r') as json_file:
                data = json.load(json_file)
            return data
        except (json.JSONDecodeError, FileNotFoundError):
            # Gérer les erreurs de décodage JSON ou de fichier non trouvé
            print("Error while reading JSON file")
            return None

    def on_modified(self, event):
        print("File +", event.src_path, "modified")
        if event.src_path == self.json_file_path:
            new_data = self.read_json()
            if new_data is not None and new_data != self.last_data:
                print("New data:", new_data)
                self.last_data = new_data

def watch_json_file(json_file_path):
    event_handler = JSONFileHandler(json_file_path)
    observer = Observer()
    observer.schedule(event_handler, path='.', recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

if __name__ == "__main__":
    json_file_path = ".\\realTimeTelemetry.json"
    watch_json_file(json_file_path)