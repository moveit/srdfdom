# Usage:
# python3 srdf_linter.py /path/to/srdf.srdf

# Command-line arguments
import sys
# For xacro loading and conversion to xml
import xacro

# For xml parsing
from xml.sax import parseString
from xml.sax.handler import ContentHandler


# For xml parsing
class SVGHandler(ContentHandler):
  def __init__(self):
    super().__init__()
    self.element_stack = []

  @property
  def current_element(self):
    return self.element_stack[-1]

  def startElement(self, name, attrs):
    self.element_stack.append({
      "name": name,
      "attributes": dict(attrs),
      "children": [],
      "value": ""
    })

  def endElement(self, name):
    clean(self.current_element)
    if len(self.element_stack) > 1:
      child = self.element_stack.pop()
      self.current_element["children"].append(child)

def clean(element):
  element["value"] = element["value"].strip()
  for key in ("attributes", "children", "value"):
    if not element[key]:
      del element[key]


class DisabledCollisionsLinter:

  def __init__(self, xacro_file):
    # Convert xacro to xml
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    # robot_description is a dict with one key: "robot_description"
    robot_description = {'robot_description': robot_description_config}

    # Create a list of all 'disable_collisions' elements
    robot_description_string = robot_description["robot_description"]
    self.xml_handler = SVGHandler()
    parseString(robot_description_string, self.xml_handler)
    xml_root = self.xml_handler.current_element
    children = xml_root["children"]

    self.disabled_collision_pairs = []

    for x in range(len(children)):
      if children[x]["name"] == "disable_collisions":
        self.disabled_collision_pairs.append([children[x]["attributes"]["link1"], children[x]["attributes"]["link2"], children[x]["attributes"]["reason"]])

  def remove_duplicate_collision_pairs(self):
    print("\nRemoving duplicate entries from the list of allowed collisions")
    n = len(self.disabled_collision_pairs)
    print("Originally there were " + str(n) + " entries in the Allowed Collision Matrix.")

    duplicate_indices = []
    for i in range(n):
      for j in range(i+1,n):
        if self.disabled_collision_pairs[i] == self.disabled_collision_pairs[j]:
          duplicate_indices.append(j)

    print("These entries are duplicated. Removing them:")
    # Remove from back to front to avoid screwing up the indices
    for i in reversed(duplicate_indices):
      print(self.disabled_collision_pairs[i][0], self.disabled_collision_pairs[i][1])
      del self.disabled_collision_pairs[i]
    n = len(self.disabled_collision_pairs)
    print("Now there are " + str(n) + " entries in the Allowed Collision Matrix.")

    # Now flip the order of the pairs. For example, [link1, link2] is a duplicate with [link2, link1]
    print("Checking if there are any duplicates when the order is reversed, e.g. (l1,l2) vs (l2,l1)")
    duplicate_indices = []
    flipped_list = []
    for i in range(n):
      flipped_list.append([self.disabled_collision_pairs[i][1], self.disabled_collision_pairs[i][0]])
    for i in range(n):
      for j in range(i+1,n):
        if flipped_list[i] == self.disabled_collision_pairs[j]:
          duplicate_indices.append(j)
    print("These entries are duplicated. Removing them:")
    for i in reversed(duplicate_indices):
      print(self.disabled_collision_pairs[i][0], self.disabled_collision_pairs[i][1])
      del self.disabled_collision_pairs[i]
    n = len(self.disabled_collision_pairs)
    print("Now there are " + str(n) + " entries in the Allowed Collision Matrix.")

  def sort_duplicate_collision_pairs(self):
    print("\nSorting the list of allowed collisions")
    self.disabled_collision_pairs = sorted(self.disabled_collision_pairs)

  def print_for_output(self):
    """Returns a well-formatted list of disabled collision pairs that can be copy/pasted into the SRDF"""
    print("\nOutput. Please copy/paste this into the SRDF:\n")
    for i in self.disabled_collision_pairs:
      print("<disable_collisions link1=\"" + i[0] + "\" link2=\"" + i[1] + "\" reason=\"" + i[2] + "\"/>")

  def print_disabled_collision_pairs(self):
    print(self.disabled_collision_pairs)
    print("\n")


def main(argv):
  if argv[0] is None:
    print("No filepath was provided")
    quit()

  print("This script will clean up issues and make suggestions for an SRDF.\n")

  inputfile = argv[0]
  disabled_collisions_linter = DisabledCollisionsLinter(inputfile)

  print("Original list of disabled collision pairs:\n")
  disabled_collisions_linter.print_disabled_collision_pairs()

  disabled_collisions_linter.remove_duplicate_collision_pairs()
  # TODO: add another check - are all adjacent links present?
  disabled_collisions_linter.sort_duplicate_collision_pairs()

  disabled_collisions_linter.print_for_output()

if __name__ == "__main__":
   main(sys.argv[1:])
