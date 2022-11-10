add `python-tk` to docker file under `RUN apt-get -y update && apt-get install -y \` in order for maze_reader to work

Also add `RUN pip install --upgrade scipy` after the apt section