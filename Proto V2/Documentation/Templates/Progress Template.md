---
tags:
  - report
  - progress
date: <% tp.date.now("YYYY-MM-DD") %>
authors:
---
<% await tp.file.rename(tp.date.now("YYYY-MM-DD") + " Progress Report") %>
# Overview
A summary of what was done today.

## Details
- list of things done today with associated attachments (and links to related notes like tests, parts, etc)