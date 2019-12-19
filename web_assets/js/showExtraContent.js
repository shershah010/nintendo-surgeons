function toggleShow(ev, id) {
  let el = document.getElementById(id);
  el.classList.toggle('show-extra');
  if (el.classList.contains('show-extra')) {
      el.scrollIntoView();
  }
}
