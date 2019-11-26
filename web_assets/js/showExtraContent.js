function toggleShow(ev, id) {
  console.log(ev);
  if (ev.srcElement.className.includes('loadbox') || ev.srcElement.offsetParent.className.includes('loadbox')) {
    document.getElementById(id).classList.toggle('show-extra');
  }
}
