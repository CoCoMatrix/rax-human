"""Host-only Gemini dollar accounting. Integer nanodollars avoid float drift."""
from decimal import Decimal, InvalidOperation, ROUND_CEILING

NANODOLLARS = 1_000_000_000
SESSION_LIMIT = 10 * NANODOLLARS
# OpenRouter prompt/completion caps are USD per million tokens; request/image
# caps are per unit. The catalog also reports this model's input-image token rate.
MAX_PRICE = {'prompt': 0.75, 'completion': 3.75, 'request': 0, 'image': 0.00000075}


def nanodollars(cost):
    if type(cost) not in (int, float):
        raise ValueError('Missing or invalid provider cost')
    try:
        value = Decimal(str(cost))
        if not value.is_finite() or value < 0:
            raise ValueError('Invalid provider cost')
        return int((value * NANODOLLARS).to_integral_value(rounding=ROUND_CEILING))
    except (InvalidOperation, OverflowError) as error:
        raise ValueError('Invalid provider cost') from error


def reserve_cost(token_reservation, max_output_tokens):
    # The existing conservative input bound includes text, image pixels and
    # protocol/schema overhead. Reasoning is covered by the output token limit.
    prompt = token_reservation - max_output_tokens
    if type(prompt) is not int or prompt < 0 or type(max_output_tokens) is not int or max_output_tokens < 1:
        raise ValueError('Invalid cost reservation')
    return (prompt * nanodollars(MAX_PRICE['prompt']) // 1_000_000
            + max_output_tokens * nanodollars(MAX_PRICE['completion']) // 1_000_000
            + 8 * nanodollars(MAX_PRICE['image']))
