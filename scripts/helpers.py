"""
Helpers when working with traces
"""

def prettify(
    original: str,
) -> str:
    """
    Process symbol to make it more readable.

    * remove std::allocator
    * remove std::default_delete
    * bind object: remove placeholder

    :param original: the original symbol
    :return: the prettified symbol
    """
    pretty = original
    # remove spaces
    pretty = pretty.replace(" ", "")
    # allocator
    std_allocator = "_<std::allocator<void>>"
    pretty = pretty.replace(std_allocator, "")
    # default_delete
    std_defaultdelete = "std::default_delete"
    if std_defaultdelete in pretty:
        dd_start = pretty.find(std_defaultdelete)
        template_param_open = dd_start + len(std_defaultdelete)
        # find index of matching/closing GT sign
        template_param_close = template_param_open
        level = 0
        done = False
        while not done:
            template_param_close += 1
            if pretty[template_param_close] == "<":
                level += 1
            elif pretty[template_param_close] == ">":
                if level == 0:
                    done = True
                else:
                    level -= 1
        pretty = pretty[:dd_start] + pretty[(template_param_close + 1) :]
    # bind
    std_bind = "std::_Bind<"
    if pretty.startswith(std_bind):
        # remove bind<>
        pretty = pretty.replace(std_bind, "")
        pretty = pretty[:-1]
        # remove placeholder stuff
        placeholder_from = pretty.find("*")
        placeholder_to = pretty.find(")", placeholder_from)
        pretty = pretty[:placeholder_from] + "?" + pretty[(placeholder_to + 1) :]
    # remove dangling comma
    pretty = pretty.replace(",>", ">")
    # restore meaningful spaces
    if pretty.startswith("void"):
        pretty = "void" + " " + pretty[len("void") :]
    if pretty.endswith("const"):
        pretty = pretty[: (len(pretty) - len("const"))] + " " + "const"

    # remove std::
    pretty = pretty.replace("std::", "")
    
    # remove void  
    pretty = pretty.replace("void ", "")

    # adding a new line
    pretty = pretty.replace("::?)(", "::?)\n(")
    
    # getting rid of the parenthesis
    pretty = pretty.replace("::?)", "::?")[1:]

    return pretty
